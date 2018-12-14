/*
    Copyright 2007 Alexandre Campo, Alvaro Gutierrez, Valentin Longchamp.

    This file is part of libIrcom.

    libIrcom is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License.

    libIrcom is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libIrcom.  If not, see <http://www.gnu.org/licenses/>.
*/

// simple test :  send or receive numbers, and avoid obstacles in the same time.

#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <stdio.h>
#include <ircom/ircom.h>
#include <btcom/btcom.h>
#include <math.h>

float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void wait(unsigned long num)
{
	while (num > 0) {num--;}
}

void obstacleAvoidance();

int main()
{
    // init robot
    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();
    e_init_motors();
    e_start_agendas_processing();

    // wait for s to start
    //btcomWaitForCommand('s');
    //btcomSendString("==== READY - IR TESTING ====\n\n");

    e_calibrate_ir();

    // initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();

while(TRUE){
    e_set_led(5,1);
    wait(1000000);
    e_led_clear();   //100000 nano sec = 100 ms
    wait(1000000);
  }
    // rely on selector to define the role
/*   int selector = getselector();
   char tmp[128];
   sprintf(tmp, "direction=%d \n", selector);
   btcomSendString(tmp);*/
    // show selector choosen

    /*e_set_speed_left(500);
    e_set_speed_right(500);
    wait(100000);
      btcomSendString("ALLUMAGE");*/


    return 0;
  }
