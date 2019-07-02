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

#include "libxtasks.h"

xtasks_stat xtasksInit()
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksFini()
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetPlatform(const char ** name)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetBackend(const char ** name)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetNumAccs(size_t * count)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetAccs(size_t const maxCount, xtasks_acc_handle * array, size_t * count)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetAccInfo(xtasks_acc_handle const handle, xtasks_acc_info * info)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksCreateTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_handle const parent, xtasks_comp_flags const compute, xtasks_task_handle * handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle * handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksAddArg(xtasks_arg_id const id, xtasks_arg_flags const flags,
    xtasks_arg_val const value, xtasks_task_handle const handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags,
    xtasks_arg_val * const values, xtasks_task_handle const handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksWaitTask(xtasks_task_handle const handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle * handle, xtasks_task_id * id)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksTryGetFinishedTaskAccel(xtasks_acc_handle const accel,
    xtasks_task_handle * task, xtasks_task_id * id)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const handle, xtasks_ins_event *events, size_t const maxCount)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksTryGetNewTask(xtasks_newtask ** task)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_handle const parent, size_t count)
{
    return XTASKS_ENOSYS;
}
