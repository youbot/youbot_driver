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

#include "generic/Logger.hpp"

namespace youbot {

  Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
  }

  void Logger::init() {
    if (this->isInitialized)
      return;
#ifdef BOOST_LOG_FOUND
    // Initialize logging to std::cout
    // std::cout << "Initializing boost::log" << std::endl;
    logging::init_log_to_console(std::cout);
    // Initialize logging to the "test.log" file
    logging::init_log_to_file("log.txt");
#endif  /* BOOST_LOG_FOUND */
    isInitialized = true;
  }

  /*
  void init()
  {
      logging::init_log_to_file
      (
          keywords::file_name = "sample_%N.log",                  // file name pattern
          keywords::rotation_size = 10 * 1024 * 1024,             // rotate files every 10 MiB...
                                                                  // ...or at midnight
          keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
          keywords::format = "[%TimeStamp%]: %_%"                 // log record format
      );

      logging::core::get()->set_filter
      (
          flt::attr< logging::trivial::severity_level >("Severity") >= logging::trivial::info
      );
  }

    // Read logging settings from a file
    //  std::ifstream file("log_settings.ini");
   //   logging::init_from_stream(file);
        // Initialize logging to std::cout
      logging::init_log_to_console(std::cout);
      // Initialize logging to the "test.log" file
      logging::init_log_to_file("log.txt");
   */
} // namespace youbot