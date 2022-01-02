/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include <cstddef>

#include "cmsis_os.h"

/* overload c memory allocator */

extern "C" void* __wrap_malloc(size_t size) { return pvPortMalloc(size); }

extern "C" void __wrap_free(void* ptr) { vPortFree(ptr); }

/* overload c++ default dynamic memory allocator */

void* operator new(size_t size) { return pvPortMalloc(size); }

void* operator new[](size_t size) { return pvPortMalloc(size); }

void operator delete(void* ptr) { vPortFree(ptr); }

void operator delete(void* ptr, unsigned int) { vPortFree(ptr); }

void operator delete[](void* ptr) { vPortFree(ptr); }

void operator delete[](void* ptr, unsigned int) { vPortFree(ptr); }
