  /*
  * read_write.c
  *
  *  Created on: 2016. 5. 16.
  *      Author: Leon Ryu Woon Jung
  */

  //
  // *********     Read and Write Example      *********
  //
  //
  // Available DXL model on this example : All models using Protocol 1.0
  // This example is designed for using a Dynamixel MX-28, and an USB2DYNAMIXEL.
  // To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below "#define"d variables yourself.
  // Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 1000000 [1M])
  //

  #ifdef __linux__
  #include <unistd.h>
  #include <fcntl.h>
  #include <termios.h>
  #elif defined(_WIN32) || defined(_WIN64)
  #include <conio.h>
  #endif

  #include <stdlib.h>
  #include <stdio.h>
  #include <iostream>

  pthread_t thread;

  void *my_printf1(void*)
  {
    // while (1)
    // {
        /* code */
        std::cout << "its first print thread!" << std::endl;
        sleep(2);
    // }
    
  }

  int main()
  {

    pthread_create(&thread, 0, &my_printf1, 0);
    sleep(1);
    while(1)
    {
        std::cout << "its main task!" << std::endl;
        sleep(2);
    }
    pthread_join(thread, 0);

    return 0;
  }