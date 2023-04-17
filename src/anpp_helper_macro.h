/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*          C Language Subsonus SDK, Version 2.4       		*/
/*              Copyright 2020, Advanced Navigation             */
/*                                                              */
/****************************************************************/
/*
 * Copyright (C) 2020 Advanced Navigation
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef ANPP_HELPER_MACRO_H
#define ANPP_HELPER_MACRO_H

#define COPY_OUT_OF_AN_PACKET(field)\
memcpy(&(field), &an_packet->data[count], sizeof((field)));\
count += sizeof((field));

#define COPY_INTO_AN_PACKET(field)\
memcpy(&an_packet->data[count], &(field), sizeof((field)));\
count += sizeof((field));

#define COPY_OUT_OF_AN_PACKET_A(field,size)\
memcpy(&(field), &an_packet->data[count], sizeof((field))*(size));\
count += (sizeof((field))*(size));

#define COPY_INTO_AN_PACKET_A(field,size)\
memcpy(&an_packet->data[count], &(field), sizeof((field))*(size));\
count += (sizeof((field))*(size));

#endif
