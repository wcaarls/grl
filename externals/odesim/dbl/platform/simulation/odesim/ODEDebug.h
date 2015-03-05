/*
 *    Copyright (C) 2012 Erik Schuitema (DBL)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *	==== DEPRECATED, PLEASE USE CLog2 FOR EASY LOGGING ====
 *
 */

#ifndef ODEDEBUG_H_
#define ODEDEBUG_H_

//#define __DEBUG__

#ifdef __DEBUG__
//TODO: If this does not work for your platform, please consult the following URL:
// http://www.redhat.com/docs/manuals/enterprise/RHEL-4-Manual/gcc/variadic-macros.html
#define dbgprintf(format, ...) fprintf (stderr, format, ## __VA_ARGS__)
#else
#define dbgprintf(format, ...) do {} while(0)
#endif


#endif /* ODEDEBUG_H_ */
