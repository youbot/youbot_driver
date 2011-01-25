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

#include <iostream>
#include <vector>
#include <signal.h>
#include <ncurses.h>

#include "youbot/YouBotBase.hpp"


using namespace std;
using namespace youbot;

bool running = true;

void sigintHandler(int signal) {
  running = false;
  printf("End!\n\r");
}

int main() {

  signal(SIGINT, sigintHandler);

  try {

    rude::Config configfile;

    if (!configfile.load("../config/applications.cfg"))
      throw FileNotFoundException("../config/applications.cfg file no found");

    int ch = 0;
    double linearVel = 0.05; //meter_per_second
    double angularVel = 0.2; //radian_per_second
    configfile.setSection("KeyboardRemoteContol");
    linearVel = configfile.getDoubleValue("TanslationalVelocity_[meter_per_second]");
    angularVel = configfile.getDoubleValue("RotationalVelocity_[radian_per_second]");

    YouBotBase myYouBotBase("youbot-base");

    JointVelocitySetpoint setVel;
    quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
    quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
    quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;

    (void) initscr(); /* initialize the curses library */
    keypad(stdscr, TRUE); /* enable keyboard mapping */
    (void) nonl(); /* tell curses not to do NL->CR/NL on output */
    (void) cbreak(); /* take input chars one at a time, no wait for \n */
    // (void) echo(); /* echo input - in color */

    def_prog_mode();

    refresh();
    printf("up = drive forward\n\r"
            "down = drive backward\n\r"
            "left = drive left\n\r"
            "right = drive right\n\r"
            "y = turn right\n\r"
            "x = turn left\n\r"
            "any other key = stop\n\r\n");
    refresh();


    while (running) {

      ch = getch();

      switch (ch) {
        case KEY_DOWN:
          longitudinalVelocity = -linearVel * meter_per_second;
          transversalVelocity = 0 * meter_per_second;
          angularVelocity = 0 * radian_per_second;
          LOG(info) << "drive backward";
          printf("\r");
          break;
        case KEY_UP:
          longitudinalVelocity = linearVel * meter_per_second;
          transversalVelocity = 0 * meter_per_second;
          angularVelocity = 0 * radian_per_second;
          LOG(info) << "drive forward";
          printf("\r");
          break;
        case KEY_LEFT:
          transversalVelocity = linearVel * meter_per_second;
          longitudinalVelocity = 0 * meter_per_second;
          angularVelocity = 0 * radian_per_second;
          LOG(info) << "drive left";
          printf("\r");
          break;
        case KEY_RIGHT:
          transversalVelocity = -linearVel * meter_per_second;
          longitudinalVelocity = 0 * meter_per_second;
          angularVelocity = 0 * radian_per_second;
          LOG(info) << "drive right";
          printf("\r");
          break;
        case 'y':
          angularVelocity = angularVel * radian_per_second;
          transversalVelocity = 0 * meter_per_second;
          longitudinalVelocity = 0 * meter_per_second;
          LOG(info) << "turn right";
          printf("\r");
          break;
        case 'x':
          angularVelocity = -angularVel * radian_per_second;
          transversalVelocity = 0 * meter_per_second;
          longitudinalVelocity = 0 * meter_per_second;
          LOG(info) << "turn left";
          printf("\r");
          break;

        default:
          longitudinalVelocity = 0 * meter_per_second;
          transversalVelocity = 0 * meter_per_second;
          angularVelocity = 0 * radian_per_second;
          LOG(info) << "stop";
          printf("\r");
          break;
      }


      myYouBotBase.setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);

      refresh();
      SLEEP_MILLISEC(100);
    }

    setVel.angularVelocity = 0 * radian_per_second;
    myYouBotBase.getBaseJoint(1).setData(setVel);
    myYouBotBase.getBaseJoint(2).setData(setVel);
    myYouBotBase.getBaseJoint(3).setData(setVel);
    myYouBotBase.getBaseJoint(4).setData(setVel);

    endwin();
    SLEEP_MILLISEC(500);

  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  } catch (...) {
    std::cout << "unhandled exception" << std::endl;
  }

  return 0;
}