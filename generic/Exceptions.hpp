/****************************************************************
 *
 * Copyright (c) 2011
 * All rights reserved.
 *
 * Hochschule Bonn-Rhein-Sieg
 * University of Applied Sciences
 * Computer Science Department
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author:
 * Jan Paulus, Nico Hochgeschwender, Michael Reckhaus, Azamat Shakhimardanov
 * Supervised by:
 * Gerhard K. Kraetzschmar
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This sofware is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Hochschule Bonn-Rhein-Sieg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ****************************************************************/

#ifndef _EXCEPTIONS_HPP
#define _EXCEPTIONS_HPP


#include <exception>
#include <stdexcept>
#include <iostream>
#include <string>

using namespace std;
namespace youbot {

		/// File not found exception
    class FileNotFoundException : public std::ios_base::failure {
        string msg;

    public:
        // Takes a character string describing the error.
        explicit FileNotFoundException(const string& message) throw ()
            :std::ios_base::failure(message) {
            msg = message + " file not found" ;
        };

        virtual ~FileNotFoundException() throw () {
        };

        // Returns a C-style character string describing the general cause of the current error
        virtual const char* what() const throw () {
            return msg.c_str();
        };
    };

		/// Key in configuration file not found exception
    class KeyNotFoundException : public std::ios_base::failure {
        string msg;

    public:
        // Takes a character string describing the error.
        explicit KeyNotFoundException(const string& message) throw ()
            :std::ios_base::failure(message) {
            msg = message + " key in config file not found" ;
        };

        virtual ~KeyNotFoundException() throw () {
        };

        // Returns a C-style character string describing the general cause of the current error
        virtual const char* what() const throw () {
            return msg.c_str();
        };
    };
		/// Joint parameter exception
    class JointParameterException : public std::runtime_error {
        string msg;

    public:
        // Takes a character string describing the error.
        explicit JointParameterException(const string& message) throw()
            : std::runtime_error(message) {
            msg = message;
        };

        virtual ~JointParameterException() throw () {
        };

        // Returns a C-style character string describing the general cause of the current error
        virtual const char* what() const throw () {
            return msg.c_str();
        };
    };
		/// Joint error exception
   class JointErrorException : public std::runtime_error {
        string msg;

    public:
        // Takes a character string describing the error.
        explicit JointErrorException(const string& message) throw ()
            :std::runtime_error(message) {
            msg = message;
        };

        virtual ~JointErrorException() throw () {
        };

        // Returns a C-style character string describing the general cause of the current error
        virtual const char* what() const throw () {
            return msg.c_str();
        };
    };
		/// EtherCAT Connection Error
   class EtherCATConnectionException : public std::runtime_error {
        string msg;

    public:
        // Takes a character string describing the error.
        explicit EtherCATConnectionException(const string& message) throw ()
            :std::runtime_error(message) {
            msg = message;
        };

        virtual ~EtherCATConnectionException() throw () {
        };

        // Returns a C-style character string describing the general cause of the current error
        virtual const char* what() const throw () {
            return msg.c_str();
        };
    };





} // namespace youbot
#endif //_EXCEPTIONS_HPP
