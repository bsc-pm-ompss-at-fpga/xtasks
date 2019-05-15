/*--------------------------------------------------------------------
  (C) Copyright 2017-2019 Barcelona Supercomputing Center
                          Centro Nacional de Supercomputacion

  This file is part of OmpSs@FPGA toolchain.

  This code is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; either version 3 of
  the License, or (at your option) any later version.

  OmpSs@FPGA toolchain is distributed in the hope that it will be
  useful, but WITHOUT ANY WARRANTY; without even the implied
  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this code. If not, see <www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <libxtasks.h>
#include <stdio.h>
#include <assert.h>

int main() {
    printf("Test01: \tCheck if initialization and finalization methods work\n");
    assert( xtasksInit() == XTASKS_SUCCESS && "Test Init function" );
    assert( xtasksFini() == XTASKS_SUCCESS && "Test Fini function" );
    printf("Test01: \tPASSED\n");
}
