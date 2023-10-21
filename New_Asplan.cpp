#include "ASplanner.h"
#include <algorithm>
#include <math.h>
#include <iostream>

using namespace std;
using namespace std::placeholders;
#define car_v 0.02;
#define acc_time 2.0;
#define de_time 2.0;
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

double ASplanner::Node::getScore()
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
int  ASplanner::Generator::time_window(vector<pathList>* paths, vector<G_Node>* GNs, vector<Gdge_property>* GEs)
{

    double time_cnt = 0;
    //初始根据根据路径长度和车辆速度来计算没有考虑碰撞的时间窗（每一段的开始时间和结束时间）
    for (auto& path : *paths) {//粗画时间窗
        time_cnt = 0;
        for (auto& GN_point : path)
        {
            GN_point.start_time = time_cnt;
            if (GN_point.index == 0) {      //0的话，是第一个节点，计算启动时间
                GN_point.spend_time = GN_point.path.leng / 100 + 2;   //???
            }
            else if (GN_point.index == -1) {    //-1的话，代表没有节点
                GN_point.spend_time = 0;
            }
            //转弯、
            //直线型
            //弯线
            //车型
            else {                  //其他的话，代表正常路段的时间
                GN_point.spend_time = GN_point.path.leng / car_v;//考虑转弯
            }
            time_cnt += GN_point.spend_time;
            GN_point.end_time = time_cnt;
        }
    }
    uint path_cnt = 0;
    uint operator_num = 0;//1为前车让后车，2为后车让前车——操作数
    for (uint i = 0; i < (*paths).size(); i++) {    // i小车数量
        if (i == 0)
        {
            continue;
        }
        //根据路径的冲突来计算时间窗的冲突，都为调整后车
        for (uint k = 0; k < ((*paths)[i]).size(); k++)//比对，待调整点。k:第i辆小车的路径节点位数
        {
            auto GN_point = &((*paths)[i][k]);//i车的第k个节点
            //auto GN_pointg = &((*paths)[i][k-1]);
            for (uint j = 0; j < i; j++)
            {
                uint size = (*paths)[j].size();
                for (auto& point_pro : (*paths)[j])//优先级更高的轨迹点
                {
                    //if ((*paths)[i][k].GN.name.main_id == point_pro.GN.name.main_id);
                    //后车的源节点等于前车的源节点 并且 后车的目标节点等于前车的目标节点
                    if (GN_point->path.source_index == point_pro.path.source_index && GN_point->path.target_index == point_pro.path.target_index)
                    {
                        /*[        ]
                                [        ]

                            [               ]
                                [        ]
                        */
                        //后车的开始时间  早于前车的结束时间 && 晚于前车的开始时间
                        if (GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time)
                        {
                            operator_num = 1;
                        }
                        /*     [        ]
                          [        ]   ->
                        */
                        //后车的结束时间  早于前车的结束时间 && 晚于前车的开始时间
                        else if (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time)
                        {
                            operator_num = 1;
                        }
                        /*     [        ]
                           [                ]
                        */
                        //后车的开始时间早于前车的开始时间 && 后车的结束时间晚于晚于前车的结束时间
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
                            //根据操作数来计算时间窗
                            for (uint m = k; m < ((*paths)[i]).size(); m++)//后续的点全部向后
                            {
                                (*paths)[i][m].start_time += set_time; //后移时间窗 
                                (*paths)[i][m].end_time += set_time;
                            }
                        }
                    }
                    /*if (GN_point->GN.name.sec_id== point_pro.GN.name.sec_id )
                    {
                        continue;
                    }*/
                    /* if (GN_point->path.source_index == point_pro.path.target_index
                         && GN_point->path.target_index == point_pro.path.source_index
                        &&point_pro_next.GN.index!=GN_pointg->GN.index)
                     {

                     }*/
                    if (GN_point->path.source_index == point_pro.path.target_index
                        && GN_point->path.target_index == point_pro.path.source_index
                        && GN_point->index != 0  //后车不为起点
                        && point_pro.index != -1) //碰撞路段不是终点路段 且不为后车的起点路段
                    {
                        if ((GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time) || (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time))
                        {
                            auto GN_pointg = &((*paths)[i][k - 1]);
                            auto pro_next = &((*paths)[j][point_pro.index + 2]);
                            if (GN_pointg->GN.index != pro_next->GN.index) {//碰撞路段后两车走不同路
                                // 后车时间窗后移
                                /*cout << "后车后一个点" << GN_pointg->GN.name.sec_id << "-" << GN_pointg->GN.name.last_id << endl
                                    << "前车下一个点" << pro_next->GN.name.sec_id << "-" << pro_next->GN.name.last_id << endl
                                    << "前车de点" << point_pro.GN.name.sec_id << "-" << point_pro.GN.name.last_id << endl
                                    << "后车de点" << GN_point->GN.name.sec_id << "-" << GN_point->GN.name.last_id << endl;*/
                                cout << "交叉路执行等待" << endl;
                                double set_time = pro_next->end_time - (*paths)[i][k].start_time;//计算后移时间

                                for (uint m = k - 1; m < ((*paths)[i]).size(); m++)//后续的点全部向后
                                {
                                    (*paths)[i][m].start_time += set_time; //后移时间窗 
                                    (*paths)[i][m].end_time += set_time;
                                }
                            }
                        }
                    }
                    if (GN_point->path.source_index == point_pro.path.target_index
                        && GN_point->path.target_index == point_pro.path.source_index
                        && GN_point->index != 0)//GN不能为起点，否则无法重新规划路线
                    {
                        if ((GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time) || (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time))
                        {
                            int a = (*paths)[i].size();
                            int index = (*paths)[i][a - 1].GN.index;//尾号
                            auto GN_pointg = &((*paths)[i][k - 1]);
                            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//删除碰撞路段的目标点
                            {
                                if (it->index == GN_pointg->GN.index)//找到上一个点
                                {           //遍历上一个点的连接关系
                                    //cout << GN_pointg->GN.name.main_id << "-" << GN_pointg->GN.name.sec_id<< "-" << GN_pointg->GN.name.last_id << "点连接关系数量" << GN_pointg->GN.link_edges.size() << endl;
                                    cout << GN_pointg->GN.name.main_id << "-" << it->name.sec_id << "-" << it->name.last_id << "点连接关系数量" << it->link_edges.size() << endl;
                                    for (int tt = 0; tt < it->link_edges.size(); tt++)
                                    {
                                        //cout << "目标点ceshi" << it->link_edges[t].target_index << endl;
                                        if (it->link_edges[tt].target_index == GN_point->path.source_index)//如果上个点目标点为该点，删除该条路径
                                        {
                                            it->link_edges[tt].state = false;
                                        }
                                    }
                                }
                            }
                            pathList xu = findPath((*GNs)[GN_pointg->path.source_index], (*GNs)[index], GNs);
                            cout << "重新规划完的路径：" << endl;
                            for (auto t = xu.begin(); t != xu.end(); t++)
                            {
                                cout << t->GN.name.main_id << "-" << t->GN.name.sec_id << "-" << t->GN.name.last_id << endl;
                            }
                            //添加时间窗
                           //删除后续路径                         
                            int c = (*paths)[i].size() - GN_point->index;
                            for (int d = 0; d < c + 1; d++)
                            {
                                (*paths)[i].pop_back();
                            }
                            int b = GN_point->index - 1;
                            int u = xu.size() - 1;
                            for (int tt = 0; tt < xu.size(); tt++)
                            {
                                if (tt == u)
                                {
                                    (*paths)[i].push_back(xu[tt]);
                                }
                                else {
                                    xu[tt].index = b;
                                    (*paths)[i].push_back(xu[tt]);
                                }
                                b++;
                            }
                            double time_cntt = 0;
                            for (auto& GN_pointg : (*paths)[i])
                            {
                                GN_pointg.start_time = time_cntt;
                                if (GN_pointg.index == 0)
                                {      //0的话，是第一个节点，计算启动时间
                                    GN_pointg.spend_time = GN_pointg.path.leng / 100 + 2;
                                }
                                else if (GN_pointg.index == -1) {    //-1的话，代表没有节点
                                    GN_pointg.spend_time = 0;
                                }
                                else {                  //其他的话，代表正常路段的时间
                                    GN_pointg.spend_time = GN_pointg.path.leng / car_v;//考虑转弯
                                }
                                time_cntt += GN_pointg.spend_time;
                                GN_pointg.end_time = time_cntt;
                            }
                        }
                    }
                }
            }
        }
        for (auto t = GNs->begin(); t != GNs->end(); t++)//恢复地图道路信息
        {
            for (auto edge : t->link_edges)
            {
                edge.state = true;
            }
        }
    }
    return 0;
}
ASplanner::pathList  ASplanner::Generator::findPath(G_Node source_, G_Node target_, vector<G_Node>* GNs)
{
    Node* current = nullptr;
    NodeSet openSet, closedSet; //openSet待选择叶节点,closedSet已经被选择的非叶节点
    openSet.reserve(100);       //申请100个元素的内存空间
    closedSet.reserve(100);     //申请100个元素的内存空间
    openSet.push_back(new Node(source_));

    while (!openSet.empty())
    {//最大可探索长度
        auto current_it = openSet.begin();
        current = *current_it;
        for (auto it = openSet.begin(); it != openSet.end(); it++)
        {
            auto node = *it;
            //查找当前节点附近最短路径的节点并将其添加到叶节点
            if (node->getScore() <= current->getScore())
            {
                current = node;
                // cout << "节点 "<<*it << " " << current->GN.name.main_id << "-" << current->GN.name.sec_id << "-" << current->GN.name.last_id << endl;
                current_it = it;
            }
        }
        //当前的坐标等于目标节点坐标，即找到目标节点
        if (current->coordinates == target_.coordinates) {
            break;
        }

        closedSet.push_back(current);   //将找到的最短路径的最近节点压入容器，即添加到叶节点
        openSet.erase(current_it);      //标记已经被选择的节点——closedSet已经被选择的非叶节点

        for (auto edge : current->GN.link_edges)//auto将指针移向当前节点的周围相邻节点
        {
            if (edge.state)
            {
                if (findNodeOnList(closedSet, edge.target_index)) {//已搜索过，跳过此点
                    continue;
                }
                //路段长度代表代价，在这里可以修改代价函数，也可加入其他的信息，符合自己需求
                double totalCost = current->G + edge.leng;
                Node* successor = findNodeOnList(openSet, edge.target_index);//是否在候选路径，不在，添加此新点；
                if (successor == nullptr)
                {//探索新点
                    successor = new Node((*GNs)[edge.target_index], current);
                    successor->G = totalCost;
                    if (fabs(current->GN.coordinates.y - successor->coordinates.y) > 0.5)
                        successor->H = heuristic(successor->coordinates, target_.coordinates) + 6;//距离代价，是欧式距离
                    openSet.push_back(successor);
                }
                else if (totalCost < successor->G) {//如果比已找到的点代价还小
                    successor->parent = current;
                    successor->G = totalCost;
                }
            }
        }
    }

    G_NodeList path;
    pathList time_path;
    while (current != nullptr) {//链表轮询
        path.push_back(current->GN);
        current = current->parent;
    }
    reverse(path.begin(), path.end());//倒叙
    for (uint i = 0; i < path.size(); i++) 
    {
        path_point temp_t;      //轨迹点索引
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
        time_path.push_back(temp_t);    //将路径的的各个站点压入time_path栈中
    }

    releaseNodes(openSet);  //释放栈空间
    releaseNodes(closedSet);//释放栈空间

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

//释放栈内所有空间
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
