#include <libxtasks.h>
#include <stdio.h>
#include <assert.h>

int main() {
    printf("Test01: \tCheck if initialization and finalization methods work\n");
    assert( xtasksInit() == XTASKS_SUCCESS && "Test Init function" );
    assert( xtasksFini() == XTASKS_SUCCESS && "Test Fini function" );
    printf("Test01: \tPASSED\n");
}
