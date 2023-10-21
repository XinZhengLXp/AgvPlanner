#include "ASplanner.h"
#include <algorithm>
#include <math.h>
#include <iostream>
using namespace std;
using namespace std::placeholders;
#define car_v 0.01;//改成全局变量
#define acc_time 0.05;
#define de_time 0.01;
bool  ASplanner::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

ASplanner::Vec2i operator + (const  ASplanner::Vec2i& left_, const  ASplanner::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

ASplanner::Node::Node(G_Node GN_, Node* parent_)
{
    parent = parent_;
    GN = GN_;
    coordinates = GN.coordinates;
    G = H = 0;
}

ASplanner::uint  ASplanner::Node::getScore()//代价函数
{
    return G + H;
}

ASplanner::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void  ASplanner::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void  ASplanner::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void  ASplanner::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void  ASplanner::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void  ASplanner::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void  ASplanner::Generator::clearCollisions()
{
    walls.clear();
}

//先规划单车路径，规划完直接计算时间窗，然后根据时间窗计算下一个车的路径,规划后续时间窗的同时考虑前面已规划车的时间窗
int ASplanner::Generator::A_star_time_window(vector<pathList>* paths, G_Node source_, G_Node target_, vector<G_Node>* GNs, vector<Gdge_property>* GEs)
{
    Node* current = nullptr;//当前节点地址
    NodeSet openSet, closedSet;//定义openlist和closelist,两者的数据类型都是Node*类型的容器
    openSet.reserve(100);//预分配内存
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));//把起点信息放入openlist内

    while (!openSet.empty()) //openlist不为空则返回True，然后取反；最大可探索长度
    {
        auto current_it = openSet.begin();//current_it是openlist的第一个节点的地址
        current = *current_it;//current是节点中的内容

        for (auto it = openSet.begin(); it != openSet.end(); it++) {//it从openlist第一个成员开始遍历，一直到openlist结尾
            auto node = *it;//取出it的内容
            if (node->state == 1)//如果点不可用
            {
                if (node->getScore() <= current->getScore()) {//如果openlist节点的代价小于当前节点，那将当前节点更改
                    current = node;//改变当前节点地址
                    current_it = it;//保存上一个节点
                }
            }
        }

        if (current->coordinates == target_.coordinates) {//找到目标节点则退出
            break;
        }

        closedSet.push_back(current);//将起点周围代价最小的点加入closelist
        openSet.erase(current_it);//擦除current_it

        for (auto edge : current->GN.link_edges)//寻找当前点的相邻边
        {
            if (findNodeOnList(closedSet, edge.target_index))
            {//已经搜索过的边则跳过，防止点返回父节点
                continue;
            }


            uint totalCost = current->G + edge.leng;//路段长度代表代价

            Node* successor = findNodeOnList(openSet, edge.target_index);//是否在候选路径，不在，添加此新点；

            if (successor == nullptr)
            {//探索新点

                successor = new Node((*GNs)[edge.target_index], current);//实例化对象，保证占用空间
                if (successor->GN.state == 0)
                {
                    delete successor;
                    continue;
                }
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_.coordinates);//距离代价
                openSet.push_back(successor);
                delete successor;
            }
            else if (totalCost < successor->G)
            {//如果比已找到的点代价还小
                if (successor->state == 0)
                {
                    delete successor;
                    continue;
                }
                successor->parent = current;
                successor->G = totalCost;
                delete successor;
            }
        }
    }

    G_NodeList path;//G_Node容器
    pathList time_path;//path_point容器
    while (current != nullptr) //链表轮询
    {
        path.push_back(current->GN);
        current = current->parent;
    }
    reverse(path.begin(), path.end());//倒叙
    for (uint i = 0; i < path.size(); i++)
    {
        path_point temp_t;
        temp_t.GN = path[i];
        if (i + 1 == path.size())
        {
            temp_t.index = -1;//代表最后一个点
        }
        else
        {
            temp_t.index = i;//轨迹点索引
            for (auto edge : temp_t.GN.link_edges)
            {
                if (edge.target_index == path[i + 1].index)//路径匹配
                {
                    temp_t.path = edge;//找到目标路径
                }
            }
        }
        time_path.push_back(temp_t);
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return 0;
}

int  ASplanner::Generator::time_window(vector<pathList>* paths, vector<G_Node>* GNs, vector<Gdge_property>* GEs)//paths是A*算法得到的所有agv路径,用于规划全部时间窗
{
    double time_cnt = 0;
    for (auto& path : *paths) {//粗画时间窗,path是每个agv的路径信息
        time_cnt = 0;
        for (auto& GN_point : path)
        {//考虑转弯情况，贝塞尔曲线怎么做，车型
            GN_point.start_time = time_cnt;
            if (GN_point.index == 0) {
                GN_point.spend_time = GN_point.path.leng / 100 + 2;
            }
            else if (GN_point.index == -1) {//最后一个节点
                GN_point.spend_time = 0;
            }
            else {
                GN_point.spend_time = GN_point.path.leng / car_v;
            }
            time_cnt += GN_point.spend_time;
            GN_point.end_time = time_cnt;
        }
    }
    uint path_cnt = 0;
    uint operator_num = 0;
    for (uint i = 0; i < (*paths).size(); i++) {//i是指第i个小车
        if (i == 0)
        {
            continue;
        }
        for (uint k = 0; k < ((*paths)[i]).size(); k++)//比对，待调整点,k是第一辆agv整条路径的每个中间点
        {
            auto GN_point = &((*paths)[i][k]);//GN_point是第i辆小车的第k个中间点
            for (uint j = 0; j < i; j++)//第j辆小车
            {
                uint l = 0;//表示第j辆小车的第l条路径
                for (auto& point_pro : (*paths)[j])//遍历第j辆小车的路径，用point_pro来表示该车的中间节点
                {
                    if (GN_point->path.source_index == point_pro.path.source_index && GN_point->path.target_index == point_pro.path.target_index)
                    {
                        /*[        ]
                                [        ]

                            [               ]
                                [        ]
                        */
                        if (GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time)
                        {
                            operator_num = 1;
                        }
                        /*     [        ]
                          [        ]   ->
                        */
                        else if (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time)
                        {
                            operator_num = 1;
                        }
                        /*     [        ]
                           [                ]
                        */
                        else if (GN_point->start_time <= point_pro.start_time && GN_point->end_time >= point_pro.end_time)
                        {
                            operator_num = 1;
                        }
                        else
                        {
                            operator_num = 0;
                        }
                        if (operator_num == 1)
                        {
                            operator_num = 0;

                            double set_time = point_pro.end_time - (*paths)[i][k].start_time;//计算后移时间
                            for (uint m = k; m < ((*paths)[i]).size(); m++)//后续的点全部向后
                            {
                                (*paths)[i][m].start_time += set_time; //后移时间窗 
                                (*paths)[i][m].end_time += set_time;
                            }
                            /* if(i==2&&k==0&&j==1)
                           {cout << GN_point->start_time <<  "\n";
                            cout << GN_point->end_time <<  "\n";
                           cout << point_pro.start_time <<  "\n";
                            cout << point_pro.end_time <<  "\n";
                          } */
                        }
                    }
                    else if(GN_point->path.source_index == point_pro.path.target_index && GN_point->path.target_index == point_pro.path.source_index)//发生相向冲突
                    {
                        /*GN_point->GN.state = 0;
                        (*paths)[i] = findPath(GN_point->GN, ((*paths)[i][(*paths)[i].size()-1]).GN, GNs, GEs);
                        time_window_dan(paths, i, GNs, GEs);
                        GN_point->GN.state = 1;*/

                        /*for (uint m = k - 1; m < ((*paths)[i]).size(); m++)//删除第i辆车的后续路径
                        {
                            ((*paths)[i]).vector::pop_back();
                        }*/

                        if ((GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time) || (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time) || (GN_point->start_time <= point_pro.start_time && GN_point->end_time >= point_pro.end_time))
                        {
                            static int count_cft_opposite = 0;//静态变量，记冲突次数
                            string temp;//用于字符串拼接的中间变量
                            ofstream ofs;//文件流

                            if (count_cft_opposite == 0)
                            {
                                ofs.open("D:\\agv_project\\Project1\\file\\conflict.txt", ios::out);//打开文件，并且设置为写入模式
                            }
                            else
                            {
                                ofs.open("D:\\agv_project\\Project1\\file\\conflict.txt", ios::app);//打开文件，并且设置为附加模式
                            }
                            //if (count_cft_opposite == 0)
                            //{
                            //    ofs << setw(3) << setfill(' ') << "High priority" << setw(15) << setfill(' ') << "Direction" << setw(30) << setfill(' ') << "Time";
                            //    ofs << setw(3) << setfill(' ') << "Low priority" << setw(15) << setfill(' ') << "Direction" << setw(30) << setfill(' ') << "Time" << endl;
                            //}
                            //在写入的同时，保证对齐
                            ofs << "conflict" << count_cft_opposite + 1 << ":";
                            ofs << setw(3) << setfill(' ') << i;
                            temp = to_string(point_pro.path.source_index) + "->" + to_string(point_pro.path.target_index);
                            ofs << setw(10) << setfill(' ') << temp;
                            //temp = to_string(point_pro.start_time) + "->" + to_string(point_pro.end_time);
                            ofs << setprecision(8) << setw(10) << setfill(' ') << point_pro.start_time << "->" << setw(10) << setfill(' ') << point_pro.end_time;
                            ofs << setw(3) << setfill(' ') << j;
                            temp = to_string(GN_point->path.source_index) + "->" + to_string(GN_point->path.target_index);
                            ofs << setw(10) << setfill(' ') << temp;
                            //temp = to_string(GN_point->start_time) + "->" + to_string(GN_point->end_time);
                            ofs << setprecision(8) << setw(10) << setfill(' ') << GN_point->start_time << "->" << setw(10) << setfill(' ') << GN_point->end_time;
                            ofs << "\n";
                            ofs.close();
                            count_cft_opposite++;
                        }
                        
                        //pathList temp_path;//记住要更改的agv的路径点
                        // auto temp_path = ASplanner::Generator::findPath(point_pro.GN.source_index, point_pro, *GNs, *GEs);//重规划第j辆车的路径
                    }
                    else if (GN_point->path.target_index == point_pro.path.target_index)//终点相同时
                    {
                        if(GN_point->end_time == point_pro.end_time)
                            operator_num = 1;

                        if (operator_num == 1)
                        {
                            operator_num = 0;

                            double set_time = 5;//计算后移时间
                            for (uint m = k; m < ((*paths)[i]).size(); m++)//后续的点全部向后
                            {
                                (*paths)[i][m].start_time += set_time; //后移时间窗 
                                (*paths)[i][m].end_time += set_time;
                            }
                            /* if(i==2&&k==0&&j==1)
                           {cout << GN_point->start_time <<  "\n";
                            cout << GN_point->end_time <<  "\n";
                           cout << point_pro.start_time <<  "\n";
                            cout << point_pro.end_time <<  "\n";
                          } */
                        }
                    }
                    l++;
                }
            }
        }
    }
    return 0;
}

int  ASplanner::Generator::time_window_dan(vector<pathList>* paths, int count, vector<G_Node>* GNs, vector<Gdge_property>* GEs)//paths是A*算法得到的所有agv路径,仅规划单车时间窗
{
    /*duble time_cnt = 0;

    auto path = (*paths)[count]; //粗画时间窗,path是每个agv的路径信息

    time_cnt = 0;
    for (auto& GN_point : path)
    {//考虑转弯情况，贝塞尔曲线怎么做，车型
        GN_point.start_time = time_cnt;
        if (GN_point.index == 0) {
            GN_point.spend_time = GN_point.path.leng / 100 + 2;
        }
        else if (GN_point.index == -1) {//最后一个节点
            GN_point.spend_time = 0;
        }
        else {
            GN_point.spend_time = GN_point.path.leng / car_v;
        }
        time_cnt += GN_point.spend_time;
        GN_point.end_time = time_cnt;
    }
    */
    uint path_cnt = 0;
    uint operator_num = 0;

    double time_cnt = 0;

    time_cnt = 0;
    auto path = (*paths)[count];

    for (uint k = 0; k < ((*paths)[count]).size(); k++)//比对，待调整点,k是第一辆agv整条路径的每个中间点
    {
        auto GN_point = &((*paths)[count][k]);//GN_point是第i辆小车的第k个中间点
        for (uint j = 0; j < count; j++)//第j辆小车
        {
            uint l = 0;//表示第j辆小车的第l条路径
            for (auto& point_pro : (*paths)[j])//遍历第j辆小车的路径，用point_pro来表示该车的中间节点
            {
                if (GN_point->path.source_index == point_pro.path.source_index && GN_point->path.target_index == point_pro.path.target_index)
                {
                    /*[        ]
                            [        ]

                        [               ]
                            [        ]
                    */
                    if (GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time)
                    {
                        operator_num = 1;
                    }
                    /*     [        ]
                        [        ]   ->
                    */
                    else if (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time)
                    {
                        operator_num = 1;
                    }
                    /*     [        ]
                        [                ]
                    */
                    else if (GN_point->start_time <= point_pro.start_time && GN_point->end_time >= point_pro.end_time)
                    {
                        operator_num = 1;
                    }
                    else
                    {
                        operator_num = 0;
                    }
                    if (operator_num == 1)
                    {
                        operator_num = 0;

                        double set_time = point_pro.end_time - (*paths)[count][k].start_time;//计算后移时间
                        for (uint m = k; m < ((*paths)[count]).size(); m++)//后续的点全部向后
                        {
                            (*paths)[count][m].start_time += set_time; //后移时间窗 
                            (*paths)[count][m].end_time += set_time;
                        }
                        /* if(i==2&&k==0&&j==1)
                        {cout << GN_point->start_time <<  "\n";
                        cout << GN_point->end_time <<  "\n";
                        cout << point_pro.start_time <<  "\n";
                        cout << point_pro.end_time <<  "\n";
                        } */
                    }
                }
                else if (GN_point->path.source_index == point_pro.path.target_index && GN_point->path.target_index == point_pro.path.source_index)//发生相向冲突
                {
                    GN_point->GN.state = 0;
                    (*paths)[count] = findPath(GN_point->GN, ((*paths)[count][(*paths)[count].size()-1]).GN, GNs, GEs);
                    time_window_dan(paths, count, GNs, GEs);
                    GN_point->GN.state = 1;
                }
                else if (GN_point->path.target_index == point_pro.path.target_index)//终点相同时
                {
                    if (GN_point->end_time == point_pro.end_time)
                        operator_num = 1;

                    if (operator_num == 1)
                    {
                        operator_num = 0;

                        double set_time = 5;//计算后移时间
                        for (uint m = k; m < ((*paths)[count]).size(); m++)//后续的点全部向后
                        {
                            (*paths)[count][m].start_time += set_time; //后移时间窗 
                            (*paths)[count][m].end_time += set_time;
                        }
                        /* if(i==2&&k==0&&j==1)
                        {cout << GN_point->start_time <<  "\n";
                        cout << GN_point->end_time <<  "\n";
                        cout << point_pro.start_time <<  "\n";
                        cout << point_pro.end_time <<  "\n";
                        } */
                    }
                }
                l++;
            }
        }
    }
    return 0;
}

ASplanner::pathList  ASplanner::Generator::findPath(G_Node source_, G_Node target_, vector<G_Node> *GNs, vector<Gdge_property> *GEs)//一次规划一辆车的路径
{
    Node* current = nullptr;//当前节点地址
    NodeSet openSet, closedSet;//定义openlist和closelist,两者的数据类型都是Node*类型的容器
    openSet.reserve(100);//预分配内存
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));//把起点信息放入openlist内

    while (!openSet.empty()) //openilst不为空则返回True，然后取反；最大可探索长度
    {
        auto current_it = openSet.begin();//current_it是openlist的第一个节点的地址
        current = *current_it;//current是节点中的内容

        for (auto it = openSet.begin(); it != openSet.end(); it++) {//it从openlist第一个成员开始遍历，一直到openlist结尾
            auto node = *it;//取出it的内容
            //if (node->state == 1)//如果点不可用
            //{
            if (node->getScore() <= current->getScore()) {//如果openlist节点的代价小于当前节点，那将当前节点更改
                current = node;//改变当前节点地址
                current_it = it;//保存上一个节点
            }
            //}
        }

        if (current->coordinates == target_.coordinates) {//找到目标节点则退出
            break;
        }

        closedSet.push_back(current);//将起点周围代价最小的点加入closelist
        openSet.erase(current_it);//擦除current_it

        for (auto edge : current->GN.link_edges)//寻找当前点的相邻边
        {
            if (findNodeOnList(closedSet, edge.target_index)) 
            {//已经搜索过的边则跳过，防止点返回父节点
                continue;
            }
     
            uint totalCost = current->G + edge.leng;//路段长度代表代价

            Node* successor = findNodeOnList(openSet, edge.target_index);//是否在候选路径，不在，添加此新点；

            if (successor == nullptr)
            {//探索新点
                
                successor = new Node((*GNs)[edge.target_index], current);//实例化对象，保证占用空间
                /*if (successor->GN.state == 0)
                {
                    delete successor;
                    continue;
                }*/
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_.coordinates);//距离代价
                openSet.push_back(successor);
                //delete successor;
            }
            else if (totalCost < successor->G)
            {//如果比已找到的点代价还小
                /*if (successor->state == 0)
                {
                    delete successor;
                    continue;
                }*/
                successor->parent = current;
                successor->G = totalCost;
                //delete successor;
            }
        }
    }

    G_NodeList path;//G_Node容器
    pathList time_path;//path_point容器
    while (current != nullptr) //链表轮询
    {
        path.push_back(current->GN);
        current = current->parent;
    }
    reverse(path.begin(), path.end());//倒叙
    for (uint i = 0; i < path.size(); i++) 
    {
        path_point temp_t;
        temp_t.GN = path[i];
        if (i + 1 == path.size()) 
        {
            temp_t.index = -1;//代表最后一个点
        }
        else 
        {
            temp_t.index = i;//轨迹点索引
            for (auto edge : temp_t.GN.link_edges) 
            {
                if (edge.target_index == path[i + 1].index)//路径匹配
                {
                    temp_t.path = edge;//找到目标路径
                }
            }
        }
        time_path.push_back(temp_t);
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return time_path;
}

ASplanner::Node* ASplanner::Generator::findNodeOnList(NodeSet& nodes_, uint index)
{
    for (auto node : nodes_) {
        if (node->GN.index == index) {
            return node;
        }
    }
    return nullptr;
}

void  ASplanner::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete* it;
        it = nodes_.erase(it);
    }
}

bool  ASplanner::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

ASplanner::Vec2i  ASplanner::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

ASplanner::uint  ASplanner::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

ASplanner::uint  ASplanner::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

ASplanner::uint  ASplanner::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
