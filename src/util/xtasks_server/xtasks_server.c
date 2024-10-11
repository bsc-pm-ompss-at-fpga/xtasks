#include <arpa/inet.h>
#include <errno.h>
#include <libxtasks.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// With big transfers, the network doesn't send everything in a single call
#define MAX_NETWORK_TRANSFER_SIZE 536870912  // 512MB

typedef enum { XTASKS_MEMCPY = 0, XTASKS_SEND_TASK = 1 } XtasksFunc;

typedef struct {
    xtasks_acc_handle handle;
    void* next;
    void* prev;
} ListElem;

static ListElem* pending_tasks_begin = NULL;
static ListElem* pending_tasks_end = NULL;

static int sockfd;
static int xtasks_connfd;
static int xdma_connfd;

static int end_server = 0;
static int connected = 0;

static void insert_list(xtasks_acc_handle handle)
{
    ListElem* newelem = malloc(sizeof(ListElem));
    newelem->handle = handle;
    newelem->next = NULL;
    newelem->prev = pending_tasks_end;
    if (pending_tasks_begin == NULL) {
        pending_tasks_begin = newelem;
    }
    if (pending_tasks_end != NULL) {
        pending_tasks_end->next = newelem;
    }
    pending_tasks_end = newelem;
}

static void delete_list(ListElem** elem_ptr)
{
    ListElem* elem = *elem_ptr;
    *elem_ptr = elem->next;
    if (elem->prev != NULL) {
        ListElem* prev = (ListElem*)elem->prev;
        prev->next = elem->next;
    }
    if (elem->next != NULL) {
        ListElem* next = (ListElem*)elem->next;
        next->prev = elem->prev;
    }
    if (elem == pending_tasks_begin) {
        pending_tasks_begin = elem->next;
    }
    if (elem == pending_tasks_end) {
        pending_tasks_end = elem->prev;
    }
    free(elem);
}

static void sig_handler(int signo)
{
    if (!connected) exit(signo);
    end_server = 1;
    printf("Signal %d caught, server will finish soon...\n", signo);
}

static int init_server(char* argv[])
{
    uint32_t port = atoi(argv[2]);

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("[SERVER] Error creating socket");
        return 1;
    }

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    inet_pton(AF_INET, argv[1], &servaddr.sin_addr);
    servaddr.sin_port = htons(port);

    int ret = bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr));
    if (ret != 0) {
        perror("[SERVER] Error binding the socket");
        close(sockfd);
        return 1;
    }

    ret = listen(sockfd, 2);
    if (ret != 0) {
        perror("[SERVER] Couldn't listen");
        close(sockfd);
        return 1;
    }

    return 0;
}

static inline size_t min(size_t a, size_t b) { return a < b ? a : b; }

static int wait_connection()
{
    xdma_connfd = accept(sockfd, NULL, 0);
    if (xdma_connfd < 0) {
        close(sockfd);
        perror("[SERVER] Error accepting");
        return 1;
    }
    xtasks_connfd = accept(sockfd, NULL, 0);
    if (xtasks_connfd < 0) {
        close(xdma_connfd);
        close(sockfd);
        perror("[SERVER] Error accepting");
        return 1;
    }

    return 0;
}

int main(int argc, char* argv[])
{
    if (argc != 3) {
        fprintf(stderr, "Usage: %s [ip] [port]\n", argv[0]);
        return 1;
    }

    if (signal(SIGINT, sig_handler) == SIG_ERR) {
        perror("[SERVER] Error handling signal");
        return 1;
    }

    if (init_server(argv) != 0) {
        return 1;
    }

    xtasks_acc_handle* ahandle;

    while (!end_server) {
        connected = 0;

        fprintf(stderr, "[SERVER] Waiting for a connection\n");
        if (wait_connection()) {
            goto error_part;
        }
        fprintf(stderr, "[SERVER] Client connected\n");

        connected = 1;

        xtasks_stat stat = xtasksInit();
        if (stat != XTASKS_SUCCESS) {
            fprintf(stderr, "[SERVER] Error initializing xtasks\n");
            close(xtasks_connfd);
            close(xdma_connfd);
            goto error_part;
        }

        size_t accCount;
        xtasksGetNumAccs(0, &accCount);
        ahandle = (xtasks_acc_handle*)malloc(sizeof(xtasks_acc_handle) * accCount);

        while (!end_server) {
            uint32_t header;
            ssize_t n;
            stat = XTASKS_PENDING;
            xtasks_task_id tid;
            do {
                usleep(1000);
                n = recv(xtasks_connfd, &header, sizeof(header), MSG_DONTWAIT | MSG_WAITALL);
                if (n < 0 && errno == EAGAIN) {
                    n = recv(xdma_connfd, &header, sizeof(header), MSG_DONTWAIT | MSG_WAITALL);
                }
                if (n < 0 && errno == EAGAIN) {
                    ListElem* it = pending_tasks_begin;
                    while (it != NULL && stat == XTASKS_PENDING) {
                        xtasks_task_handle thandle;
                        stat = xtasksTryGetFinishedTaskAccel(it->handle, &thandle, &tid);
                        if (stat == XTASKS_SUCCESS) {
                            delete_list(&it);
                            xtasksDeleteTask(&thandle);
                            break;
                        } else {
                            it = it->next;
                        }
                    }
                }
            } while (n < 0 && errno == EAGAIN && stat == XTASKS_PENDING && !end_server);
            if (end_server) break;
            if (stat != XTASKS_PENDING) {
                if (stat != XTASKS_SUCCESS) {
                    fprintf(stderr, "[SERVER] Error getting finished task\n");
                    goto error_all;
                } else {
                    uint64_t data = tid;
                    n = send(xtasks_connfd, &data, sizeof(data), 0);
                    if (n < 0) {
                        perror("[SERVER] Error in send");
                        goto error_all;
                    }
                    continue;
                }
            } else if (n < 0) {
                perror("[SERVER] Error in recv");
                goto error_all;
            } else if (n != sizeof(header)) {
                if (n == 0) {
                    fprintf(stderr, "[SERVER] Client disconnected\n");
                    break;
                } else {
                    fprintf(stderr, "[SERVER] Expected to receive %lu bytes received %ld\n", sizeof(header), n);
                    goto error_all;
                }
            }

            switch (header & 0xFF) {
                case (int)XTASKS_MEMCPY: {
                    uint64_t data[4];
                    n = recv(xdma_connfd, data, sizeof(data), MSG_WAITALL);
                    if (n != sizeof(data)) {
                        fprintf(stderr, "[SERVER] Expected to receive %lu bytes received %ld\n", sizeof(data), n);
                        goto error_all;
                    }
                    size_t len = data[2];
                    void* alloc_handle = malloc(data[0]);
                    if (alloc_handle == NULL) {
                        fprintf(stderr, "[SERVER] Error allocating memory\n");
                        goto error_all;
                    }
                    n = recv(xdma_connfd, alloc_handle, data[0], MSG_WAITALL);
                    if ((size_t)n != data[0]) {
                        fprintf(stderr, "[SERVER] Expected to receive %lu bytes received %ld\n", data[0], n);
                        goto error_all;
                    }
                    void* usr = malloc(data[2]);
                    if (usr == NULL) {
                        fprintf(stderr, "[SERVER] Error allocating memory\n");
                        free(alloc_handle);
                        goto error_all;
                    }
                    xtasks_memcpy_kind mode = data[3] == 0 ? XTASKS_HOST_TO_ACC : XTASKS_ACC_TO_HOST;
                    if (mode == XTASKS_HOST_TO_ACC) {
                        for (uint64_t i = 0; i < len; i += MAX_NETWORK_TRANSFER_SIZE) {
                            size_t t = min(MAX_NETWORK_TRANSFER_SIZE, len - i);
                            n = recv(xdma_connfd, (void*)((uintptr_t)usr + i), t, MSG_WAITALL);
                            if ((size_t)n != t) {
                                fprintf(stderr, "[SERVER] Expected to receive %lu bytes received %ld\n", t, n);
                                free(alloc_handle);
                                free(usr);
                                goto error_all;
                            }
                        }
                    }
                    stat = xtasksMemcpy((const xtasks_mem_handle)alloc_handle, data[1], data[2], usr, mode);
                    if (mode == XTASKS_HOST_TO_ACC) {
                        uint32_t ret = stat;
                        n = send(xdma_connfd, &ret, sizeof(ret), 0);
                        if (n != sizeof(stat)) {
                            fprintf(stderr, "[SERVER] Expected to send %lu bytes sent %ld\n", sizeof(stat), n);
                            free(alloc_handle);
                            free(usr);
                            goto error_all;
                        }
                    } else {
                        for (uint64_t i = 0; i < len; i += MAX_NETWORK_TRANSFER_SIZE) {
                            size_t t = min(MAX_NETWORK_TRANSFER_SIZE, len - i);
                            n = send(xdma_connfd, (void*)((uintptr_t)usr + i), t, 0);
                            if (n < 0) {
                                perror("[SERVER] Error in send");
                                free(alloc_handle);
                                free(usr);
                                goto error_all;
                            } else if ((size_t)n != t) {
                                fprintf(stderr, "[SERVER] Expected to send %lu bytes but found %ld\n", t, n);
                                free(alloc_handle);
                                free(usr);
                                goto error_all;
                            }
                        }
                    }

                    free(alloc_handle);
                    free(usr);
                    break;
                }
                case (int)XTASKS_SEND_TASK: {
                    int nargs = header >> 16;
                    int nCommands = nargs * 2 + 3;
                    int size = nCommands * sizeof(uint64_t);
                    uint64_t data[34];  // Nargs max is 15
                    n = recv(xtasks_connfd, data, size, MSG_WAITALL);
                    if (n != size) {
                        fprintf(stderr, "[SERVER] Expected to receive %d bytes but found %ld\n", size, n);
                        goto error_all;
                    }
                    uint64_t task_data[2];
                    n = recv(xtasks_connfd, task_data, sizeof(task_data), MSG_WAITALL);
                    if (n < 0) {
                        perror("[SERVER] Error in recv");
                        goto error_all;
                    } else if (n != sizeof(task_data)) {
                        fprintf(stderr, "[SERVER] Expected to receive %lu bytes but found %ld\n", sizeof(task_data), n);
                        goto error_all;
                    }

                    int devId = task_data[0] & 0xFFFFFFFF;
                    int accId = task_data[0] >> 32;
                    size_t count;
                    xtasksGetAccs(devId, accCount, ahandle, &count);
                    xtasks_task_handle thandle;
                    xtasksCreateTask(task_data[1], ahandle[accId], 0, XTASKS_COMPUTE_ENABLE, &thandle);
                    for (int i = 0; i < nargs; ++i) {
                        xtasksAddArg(i, 0xFF, data[3 + i * 2 + 1], thandle);
                    }
                    stat = xtasksSubmitTask(thandle);
                    if (stat != XTASKS_SUCCESS) {
                        fprintf(stderr, "[SERVER] Could not submit task\n");
                        goto error_all;
                    }
                    insert_list(ahandle[accId]);
                    break;
                }
                default: {
                    fprintf(stderr, "[SERVER] Unknown header 0x%X\n", header);
                    goto error_all;
                }
            }
        }
        close(xtasks_connfd);
        close(xdma_connfd);
        xtasksFini();
        free(ahandle);
        if (pending_tasks_begin != NULL || pending_tasks_end != NULL) {
            fprintf(stderr, "Client disconnected but pending task list is not empty!\n");
            goto error_part;
        }
    }

    close(sockfd);

    return 0;
error_all:
    free(ahandle);
    xtasksFini();
    close(xtasks_connfd);
    close(xdma_connfd);
    close(sockfd);
    return 1;
error_part:
    close(sockfd);
    return 1;
}
