#include <libxtasks.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

int main() {
    size_t cnt;
    xtasks_acc_handle array[2];
    xtasks_acc_info info;

    printf("Test02: \tCheck if accelerators info can be obtained\n");
    assert( xtasksInit() == XTASKS_SUCCESS && "Test Init function" );

    assert( xtasksGetNumAccs(&cnt) == XTASKS_SUCCESS && "Test num detected accels" );
    assert( cnt == 2 && "Test num detected accels" );

    assert( xtasksGetAccs(1, &array[0], &cnt) == XTASKS_SUCCESS && "Test get accel handle");
    assert( cnt == 1 && "Test get accel handle" );

    assert( xtasksGetAccInfo(array[0], &info) == XTASKS_SUCCESS && "Test get accel info");
    assert( info.id == 0 && "Test get accel info" );
    assert( info.type == 0 && "Test get accel info" );
    assert( strcmp(info.description, "foo0") == 0 && "Test get accel info" );

    assert( xtasksGetAccs(2, &array[0], &cnt) == XTASKS_SUCCESS && "Test get all accel handle");
    assert( cnt == 2 && "Test get all accel handle" );

    assert( xtasksGetAccInfo(array[0], &info) == XTASKS_SUCCESS && "Test get accel_0 info");
    assert( info.id == 0 && "Test get accel_0 info" );
    assert( info.type == 0 && "Test get accel_0 info" );
    assert( strcmp(info.description, "foo0") == 0 && "Test get accel_0 info" );

    assert( xtasksGetAccInfo(array[1], &info) == XTASKS_SUCCESS && "Test get accel_1 info");
    assert( info.id == 1 && "Test get accel_1 info" );
    assert( info.type == 1 && "Test get accel_1 info" );
    assert( strcmp(info.description, "foo1") == 0 && "Test get accel_1 info" );

    assert( xtasksFini() == XTASKS_SUCCESS && "Test Fini function" );
    printf("Test02: \tPASSED\n");
}
