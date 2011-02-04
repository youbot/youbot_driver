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

#ifndef YOUBOT_LOGGER_HPP
#define	YOUBOT_LOGGER_HPP

#include <iostream>
#include <fstream>

#include "boost/date_time/posix_time/posix_time.hpp"


namespace youbot {

	enum severity_level {
		trace,
		debug,
		info,
		warning,
		error,
		fatal
	};

	class PrintOut {
	private:
		static const bool toConsole = true;
		static const bool toFile = false;
		static const severity_level logginLevel = trace;

		std::stringstream out;
		bool print;
	public:

		PrintOut(const std::string &funcName, const int &lineNo, const std::string &fileName, severity_level level);
		~PrintOut();

		template <class T>
		PrintOut::PrintOut & operator<<(const T &v) {
			out << v;
			return *this;
		}
	};


#define LOG(level) PrintOut(__PRETTY_FUNCTION__, __LINE__ , __FILE__, level)

} // namespace youbot

#endif	/* YOUBOT_LOGGER_HPP */
