#include <iostream>
#include "Log.h"
#include <sys/time.h>
#include <time.h>
#include <ctime>
#include <unistd.h>
// #include "utils.hpp"

using namespace std;
using namespace stcontroller;

int main()
{

    cout << " log system." << endl;
    vector<string> file_name = {"test1", "test0"};
    const vector<string> data_name = {"data1", "data2"};
    vector<float>  data; 
    data.push_back(0);
    data.push_back(1);
    cout << " name:"<< file_name[0] << endl;
    cout << " name:"<< file_name[1] << endl;
    Log logSystem(file_name, "/home/cc/Desktop/Git_Folder/krock1_NHY/workspace/experiment_data/");
    logSystem.saveData(file_name[0], data, data_name);
    logSystem.step();

    for (int  i = 0; i < 50; i++)
    {
        data[0] += 2;data[1] += 2;
        logSystem.saveData(file_name[0], data, data_name);
        logSystem.step();
        cout << "loop times: " << endl;
        cout << "data[0]: " << data[0] <<endl;
        cout << "data[1]: " << data[1] <<endl;
        usleep(200000);
    }
    return 0;
}