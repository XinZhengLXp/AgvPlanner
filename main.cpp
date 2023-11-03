#include <iostream>
#include "ASplanner.h"
#include "tinyxml2.h"
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <stdio.h>
#include <fstream>
#include <string>
#include <algorithm>
#include <time.h>
#include<chrono>
using namespace tinyxml2;
using namespace std;
using namespace  ASplanner;
const double work_consum = 5.0;
vector<pair<Car_config,pathList>> GNLs;
ifstream  fin;  //声明一个ofstream对象，用于输入文件
//ofstream fout;  //声明一个ofstream对象，用以输出文件
#pragma warning(disable:4996)
vector<G_Node> GNs;

vector<Gdge_property>GEs ;

int  findpoint(const char* point)
{
    char str[20] = {0};
    const char* ptr = point;
    int x = 0;
    while (*ptr != '\0') {
        char currentChar = *ptr; // 使用解引用操作符获取当前字符
        str[x] = currentChar;
        ptr++; // 指针指向下一个字符
        x++;
    }
    vector<string> s;
    char* next_token;
    char* token = strtok_s(str, "-", &next_token);
    while (token != NULL) {
        char str[20] = { 0 };
        const char* ptr = token;
        int x = 0;
        while (*ptr != '\0') {
            char currentChar = *ptr; // 使用解引用操作符获取当前字符
            str[x] = currentChar;
            ptr++; // 指针指向下一个字符
            x++;
        }
        s.push_back(str);
        token = strtok_s(NULL,"-", &next_token);
    }
    //cout<<"起始/目标点" << s[0] << "-" << s[1] << "-" << s[2] << endl;
    vector<G_Node>::iterator it;
    int i = 0;
    for (it = GNs.begin(); it != GNs.end(); it++)
    {
        if (it->name.main_id == atoi(s[0].c_str()) && it->name.sec_id == atoi(s[1].c_str()) && it->name.last_id == atoi(s[2].c_str()))
        {
            i = it->index;
            break;
        }
    }
    return i;
}

int main()
{
    /*std::string inputFileName = "./test_example/car_config_4cars/car_config_04_01.xml";*/
    std::string inputFileName = "./config/car_config.xml";
    std::size_t found = inputFileName.find_last_of(".");
    std::string outputFileName = inputFileName.substr(0, found);
    char* p = &inputFileName[0];
    auto start = std::chrono::high_resolution_clock::now();
    XMLDocument doc_agv;
    tinyxml2::XMLError if_success=doc_agv.LoadFile(p);
   /* if (if_success != 0) { 
        std::cout << "配置文件打开失败！" << endl;
        ; exit(1);
    }*/
    
    doc_agv.Error();
    //读取车辆属性
    ofstream fout;
    fout.open(outputFileName + ".txt", ios::out); //打开car_plan.txt文件并进行输出
    
    ofstream fout2;
    fout2.open(outputFileName + "_index.txt", ios::out);
    XMLElement* agv_titleElement = doc_agv.FirstChildElement();
    XMLElement* agv = agv_titleElement->FirstChildElement("agv");
    GEs.reserve(620);
    ifstream ifse;
    ifse.open("./file/segment.txt", ios::in);
    char buf[1024];
    while (ifse.getline(buf, sizeof(buf)))
    {
        Gdge_property temp_GE;
        vector<string> s;
        char* next_token;
        char* token = strtok_s(buf, " ", &next_token);
        while (token != NULL) {
            const char* ptr = token;
            char st[100] = { 0 };
            int x = 0;
            while (*ptr != '\0')
            {
                char currentChar = *ptr; // 使用解引用操作符获取当前字符
                st[x] = currentChar;
                ptr++;
                x++;
            }
            s.push_back(st);
            token = strtok_s(NULL, " ", &next_token);
        }
        temp_GE.source_index = atoi(s[1].c_str());
        temp_GE.target_index = atoi(s[0].c_str());
        temp_GE.leng = stod(s[2].c_str());
        //cout << temp_GE.leng << endl;      double类型
        GEs.push_back(temp_GE);
        s.clear();
    }

    ifstream ifsc;
    ifsc.open("./file/output.txt", ios::in);
    char bufc[1024];
    while (ifsc.getline(bufc, sizeof(bufc)))
    {
        if (bufc != "\n")
        {
            vector<string> s;
            char* next_token;
            char* token = strtok_s(bufc, " ", &next_token);
            while (token != NULL) {
                const char* ptr = token;
                char st[100] = { 0 };
                int x = 0;
                while (*ptr != '\0')
                {
                    char currentChar = *ptr; // 使用解引用操作符获取当前字符
                    st[x] = currentChar;
                    ptr++;
                    x++;
                }
                s.push_back(st);
                token = strtok_s(NULL, " ", &next_token);
            }
            for (auto& GE : GEs)
            {
                if(GE.source_index==atoi(s[0].c_str())&& GE.target_index == atoi(s[1].c_str()))
                { 
                    GE.collsion_id = atoi(s[2].c_str());
                }
            }
            s.clear();
        }
    }

    ifsc.close();
      
    ifse.close();
    ifstream ifst;
    ifst.open("./file/station.txt", ios::in);
    char buff[1024];
    while (ifst.getline(buff, sizeof(buff)))
    {
        G_Node temp_GN;
        Gdge_property temp_GE;
        vector<string> s;
        char* next_token;
        char* token = strtok_s(buff," ", &next_token);
        while (token != NULL) {
            const char* ptr = token;
            char st[100] = { 0 };
            int x = 0;
            while (*ptr != '\0')
            {
                char currentChar = *ptr; // 使用解引用操作符获取当前字符
                st[x] = currentChar;
                ptr++;
                x++;
            }
            //cout << st << endl;
            s.push_back(st);
            token = strtok_s(NULL, " ", &next_token);
        }
        temp_GN.coordinates.x = stod(s[0].c_str());
        temp_GN.coordinates.y = stod(s[1].c_str());
        temp_GN.name.main_id = atoi(s[2].c_str());
        temp_GN.name.sec_id = atoi(s[3].c_str());
        temp_GN.name.last_id = atoi(s[4].c_str());
        temp_GN.index = atoi(s[5].c_str());
        s.clear();
        for (auto GE : GEs)
        {
            if (GE.source_index == temp_GN.index)//源节点匹配
            {
                temp_GN.link_edges.push_back(GE);
            }
        }
        GNs.push_back(temp_GN);
    }
    ifst.close();

    

    ASplanner::Generator generator;
    //generator.setWorldSize({ 25, 25 });//设置地图大小,没有太大意义，可将此行注释掉
    generator.setHeuristic(ASplanner::Heuristic::euclidean);//测距函数，使用仍然是欧氏距离
    
    int count = 1;
    while (agv) //轮询，读取车辆信息
    {
        Car_config car = {};
        pair<Car_config, pathList> temp_path;
        //temp_path.second.reserve(100);
        const XMLAttribute* start_point = agv->FindAttribute("start_point");
        const XMLAttribute* target_point = agv->FindAttribute("target_point");
        const XMLAttribute* end_point = agv->FindAttribute("end_point");
        const XMLAttribute* type = agv->FindAttribute("type");
        const XMLAttribute* car_v = agv->FindAttribute("car_v");
        XMLElement* middle_point = agv->FirstChildElement("middle_point");
        //car.target_index =findpoint(target_point->Value());
        car.type = std::atoi(type->Value());
        car.car_v = std::stod(car_v->Value());
        car.index = count;

        temp_path.first = car;
        // cout << "终点索引" << findpoint(end_point->Value()) << endl;
         //{    int index_forward;
         //     int index_backward;//下个点
         //     double x, y;
         //     //以上信息需要给出
         //     double length_for=pow(std::pow(GNs[index_forward].coordinates.x,2)+ std::pow(GNs[index_forward].coordinates.y, 2),0.5);
         //     double length_back = pow(std::pow(GNs[index_backward].coordinates.x, 2) + std::pow(GNs[index_backward].coordinates.y, 2), 0.5);
         //     Gdge_property temp1 = {true,0,length_for,GNs.size()+1,index_forward };
         //    Gdge_property temp2= { true,0,length_back,GNs.size()+ 1,index_backward };
         //    vector<Gdge_property> gdge = { temp1 ,temp2};
         //    G_Node new_node = { {0,0,0},GNs.size() + 1,{x,y}, gdge};
         //    GNs.push_back(new_node);

         //    for (auto it = GNs.begin(); it != GNs.end(); it++)
         //    {       
         //        if (it->index == index_forward)
         //        {
         //            Gdge_property temp = {true,0,index_forward ,GNs.size() + 1 };
         //            it->link_edges.push_back(temp);
         //        }
         //        if (it->index == index_backward)
         //        {
         //            Gdge_property temp = { true,0,index_backward,GNs.size() + 2 };
         //            it->link_edges.push_back(temp);
         //        }
         //    }
         //    pathList pathone=generator.findPath(GNs[findpoint(start_point->Value())], GNs[car.target_index], &GNs);
         //    pathList pathsec= generator.findPath(GNs[car.target_index], GNs[findpoint(end_point->Value())], &GNs);
         //    std::merge(pathone.begin(), pathone.end(), pathsec.begin(), pathsec.end(), temp_path.second.begin());
         //}
         // 
         //第i个中间节点,遍历结束时一共有i个中间节点
        car_path single_path;
        single_path.first = car;
        uint point_index = findpoint(start_point->Value());
        double time_cnt = 0.0;
        while (middle_point)
        {
            const XMLAttribute* middlepoint = middle_point->FindAttribute("name");
            //cout << point_index << endl;

            int target_p = findpoint(middlepoint->Value());               //目标点
           single_path.second= generator.findPath(GNs[point_index], GNs[target_p], &GNs);
            //point_index为起点                 
            generator.init_time_windows(time_cnt, &single_path, &GNs);

            uint single_size = single_path.second.size();           //单段路的长度
            uint index = single_path.second[single_size - 1].GN.index;  //最后一个点的索引号

            single_path.second[single_size - 1].GN.index = index;        //每段路的最后一个点初始化
            single_path.second[single_size - 1].path.source_index = index;
            single_path.second[single_size - 1].path.target_index = index;
            single_path.second[single_size - 1].start_time = single_path.second[single_size - 2].end_time;
            single_path.second[single_size - 1].spend_time = work_consum;
            single_path.second[single_size - 1].end_time = single_path.second[single_size - 1].start_time + work_consum;

            time_cnt = single_path.second[single_size - 1].end_time;    //更新时间

            pair<int, int>temp_pair(temp_path.second.size(), point_index);//起点的（索引号，索引号）
            temp_path.first.middle_point.push_back(temp_pair);    //记录该段路的起点信息

            //temp_path中加入该节路径
            for (uint m = 0; m < single_size; m++) {
                temp_path.second.push_back(single_path.second[m]);
            }
            single_path.second.clear();

            temp_pair = { temp_path.second.size() - 1, target_p };     //记录该段路的终点信息
            temp_path.first.middle_point.push_back(temp_pair);

            point_index = target_p;         //更新起点信息
            middle_point = middle_point->NextSiblingElement("middle_point");
        }
    
         //最后一段路
        single_path.second = generator.findPath(GNs[point_index], GNs[findpoint(end_point->Value())], &GNs);

        pair<int, int>temp_pair(temp_path.second.size(), point_index);
        temp_path.first.middle_point.push_back(temp_pair);

        generator.init_time_windows(time_cnt,&single_path,&GNs);
        
        for (uint m = 0; m < single_path.second.size(); m++) {
            temp_path.second.push_back(single_path.second[m]);
        }
        single_path.second.clear();

        temp_pair = { temp_path.second.size() - 1, findpoint(end_point->Value()) };
        temp_path.first.middle_point.push_back(temp_pair);

        uint size = temp_path.second.size();
        for (uint j = 0; j < size;j++) {
            temp_path.second[j].index = j;
            if (j == size - 1) { temp_path.second[j].index = -1; }
        }
        //cout << temp_path.first.middle_point.size() << endl;

        GNLs.push_back(temp_path);  //每一个AGV生成的轨迹
        agv = agv->NextSiblingElement("agv");
        count++;
    }


    //冲突检测
    generator.conflict_check(&GNLs, &GNs);

    for (auto it = GNLs.begin(); it != GNLs.end();) {//删除空元素
        if (it->first.index == -1) {
            it = GNLs.erase(it);
        }
        else { ++it; }
    }
    /*cout << GNLs[0].second.size() << endl;*/

    //for (auto it = (*GNLs.begin()).first.middle_point.begin(); it != (*GNLs.begin()).first.middle_point.end();it++) {//删除空元素
    //    cout << (*it).first << " " << (*it).second << endl;
    //}

    //cout << GNLs.size() << endl;
    //重新排序
    sort(GNLs.begin(), GNLs.end(),[](const pair<Car_config,pathList>& a, pair<Car_config, pathList>& b) {
            return a.first.index < b.first.index;
        });

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "system consum time: " << duration.count() << std::endl;
    for (uint i = 0; i < GNLs.size(); i++)
    {
        for (auto& GN_point : GNLs[i].second) {
            if (GN_point.index == -1)
                break;
            fout << GNs[(GN_point.path.source_index)].name.main_id <<"-"
                << GNs[(GN_point.path.source_index)].name.sec_id << "-"
                << GNs[(GN_point.path.source_index)].name.last_id<< " " 
                <<"->" << GNs[(GN_point.path.target_index)].name.main_id << "-"
                << GNs[(GN_point.path.target_index)].name.sec_id << "-"
                << GNs[(GN_point.path.target_index)].name.last_id << "\t" 
                <<(GN_point.start_time)<<"->"<< (GN_point.end_time)<<" "<< (GN_point.spend_time)
                << endl;
        }
        fout << "car" << i + 1 << ":" << endl;
    }
    fout.close();
    for (uint i = 0; i < GNLs.size(); i++)
    {
        for (auto& GN_point : GNLs[i].second) {
            if (GN_point.index == -1)
                break;
            fout2 << (GN_point.path.source_index) << "->" << (GN_point.path.target_index) << " " <<
             (GN_point.start_time) << "->" << (GN_point.end_time)
             << " " << (GN_point.spend_time) << endl;
        }
        fout2 << "car" << i + 1 << ":" << endl;
    }
    fout2.close();
    
}
