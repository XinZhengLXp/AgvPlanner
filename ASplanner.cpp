#include "ASplanner.h"
#include <algorithm>
#include <math.h>
#include <iostream>

using namespace std;
using namespace std::placeholders;
const double fork_length = 1.0;
const double agv_length = 0.5;
const double fork_acc =0.8;
const double agv_acc = 1.0;
const double de_time =2.0;   
const double wheel_time=1.0;
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

double ASplanner::Generator::time_window(vector<pair<Car_config,pathList>>* paths, vector<G_Node>* GNs)
{
    vector<pair<Car_config, ID>>collection;
    double time_cnt = 0;
    for (auto& path : *paths) {//粗画时间窗
        path.second = init_time_windows(&path,GNs);
    }

    //for (int i = 0; i < (*paths).size(); i++)
    //{
    //    if ((*paths)[i].second[0].GN.name.last_id != 0)
    //    {
    //        pair<Car_config, ID> temp_pair;
    //        temp_pair.first = (*paths)[i].first;
    //        temp_pair.second = (*paths)[i].second[0].GN.name;
    //        collection.push_back(temp_pair);
    //    }
    //}

    //for (int i = 0; i < (*paths).size(); i++)
    //{
    //    uint size = (*paths)[i].second.size();
    //    bool flag = false;
    //    uint j = 0;
    //    for(j;j<(*paths)[i].second.size();j++)
    //    { 
    //        if ((*paths)[i].second[j].GN.name.last_id == 0
    //            && (*paths)[i].second[j].index !=-1)//该点不为最后一个点
    //        {
    //            if ((*paths)[i].second[j+1].GN.name.last_id != 0)//下一个点为断头路
    //            {
    //                flag = true;
    //                break;
    //            }
    //        }
    //        j++;
    //    }
       /* uint a = size - j;
        if (flag)
        {
            for (auto it = collection.begin(); it != collection.end(); it++)
            {
                if(it->second.main_id ==(*paths)[i].second[j].GN.name.main_id 
                    && it->first.index != (*paths)[i].first.index)
                {
                    if (it->second.sec_id == (*paths)[i].second[j + 1].GN.name.sec_id )
                    {
                        for (uint count = 0; count < a; count++)
                        {
                            (*paths)[i].second.pop_back();
                        }
                    }
                    (*paths)[i].second[(*paths)[i].second.size() - 1].index = -1;
                    break;
                }
            } 
        }
    }*/
    //cout << "车一长度：" << (*paths)[0].second.size() << endl;
    for (uint i = 0; i < (*paths).size(); i++) {    // i小车数量
        if (i == 0)
        {
            continue;
        }
        uint size = (*paths)[i].second.size();
        bool flag = true;
        int abc = 0;
        while (flag) {
            flag = false;
            abc++;
            for (uint k = 0; k < (*paths)[i].second.size() && !flag; k++)//比对，待调整点。k:第i辆小车的路径节点位数
            {
                auto GN_point = &((*paths)[i].second[k]);//i车的第k个节点
                for (uint j = 0; j < i; j++)
                {
                    if (flag)
                    {
                        break;
                    }
                    for (uint n = 0; n < (*paths)[j].second.size();n++)//优先级更高的轨迹点
                    {
                       // 节点冲突
                        if ((*paths)[j].second.size() >2 ) {
                            pathList new_node_c_way = node_conflict(&(*paths)[i],k,&(*paths)[j],n, GNs);
                            (*paths)[i].second = new_node_c_way;
                        }

                        //相向冲突
                        if ((*paths)[i].second.size() > 2)
                        {   pathList new_way=opposing_conflict(&(*paths)[i], k, &(*paths)[j], n, GNs);
                            if(new_way.size()!=0){
                                (*paths)[i].second = new_way;
                                flag = true;
                                break;
                            } 
                        }
                        /*pathList new_way = station_is_vechel(k, pro_size, &point_pro, &(*paths)[i], GNs);
                        if (new_way.size() != 0)
                        {
                            (*paths)[i].second = new_way;
                            flag = true;
                            break;
                        }*/
                        //冲突集判断及其策略
                        pathList new_collsion_way = collsion_collection(&(*paths)[i], k, &(*paths)[j], n, GNs);
                        if (new_collsion_way.size() != 0)
                        {
                            (*paths)[i].second = new_collsion_way;
                            flag = true;
                            break;
                        }
                        
                        car_path new_colliding_way = colliding_conflict(&(*paths)[i], k, &(*paths)[j], n, GNs);
                        if (new_collsion_way.size() != 0)
                        {
                            (*paths)[i] = new_colliding_way;
                            flag = true;
                            break;
                        }
                    }
                }//车m
            }//车i的k
        }//while循环
        std::cout<<i+1<<" 车" << "while循环次数 ：" << abc << endl;
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
                    successor->H = heuristic(successor->coordinates, target_.coordinates)/100;
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
    for (uint i = 0; i < path.size(); i++) {
        path_point temp_t;      //轨迹点索引
        temp_t.GN = path[i];
        if (i + 1 == path.size()) {
            temp_t.index = -1;//代表最后一个点
        }
        else {
            temp_t.index = i;//轨迹点索引
            for (auto edge : temp_t.GN.link_edges) {
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
    if (time_path[time_path.size()-1].GN.index != target_.index)
    {
        cout << "寻路失败！" << endl;
        time_path = {};
    }
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
//相向冲突
ASplanner::pathList ASplanner::Generator::opposing_conflict(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs)
{
    pathList new_way = {};
    auto point_pro = &(*pro_path).second[n];
    if ((*path).second[k].GN.index == point_pro->path.target_index
        && (*path).second[k].path.target_index == point_pro->path.source_index
        && (*path).second[k].index != 0  //不为第一段
        &&  path->second.size() > 2
        )
    {
       // double length_time = 0.0;
        //if (pro_path->first.type == 0) { length_time = agv_length / (*pro_path).first.car_v; }//AGV车长/速度
        //else { length_time = fork_length / (*pro_path).first.car_v; }//叉车车长/速度

        if (((*path).second[k].start_time < point_pro->end_time && (*path).second[k].start_time > point_pro->start_time)
            || ((*path).second[k].end_time < point_pro->end_time  && (*path).second[k].end_time > point_pro->start_time))
        {
            size_t path_size = path->second.size();
            int start_index = ((*path).second[path_size - 1].GN.index);  //最后一个元素索引号地址
            int end_index = ((*path).second[0].GN.index);
            auto GN_pointg = &(*path).second[k - 1];

            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//删除碰撞路段的目标点
            {
                if (it->index == GN_pointg->GN.index)
                {
                    for (int tt = 0; tt < it->link_edges.size(); tt++)
                    {
                        if (it->link_edges[tt].target_index == GN_pointg->path.target_index)//如果上个点目标点为该点，删除该条路径
                        {
                            it->link_edges[tt].state = false;
                        }
                    }
                }
            }
            (*path).second = findPath((*GNs)[start_index], (*GNs)[end_index], GNs);
            
            std::cout << "相向冲突规划完成" << endl;
            (*path).second = init_time_windows(&(*path), GNs);
            new_way = (*path).second;
        }
    }
    return new_way;
}
//冲突集
ASplanner::pathList ASplanner::Generator::collsion_collection(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs)
{
     pathList new_way = {};
     auto point_pro = &(*pro_path).second[n];
     auto GN_point = &(*path).second[k];
    if (GN_point->path.collsion_id == point_pro->path.collsion_id &&GN_point->path.collsion_id != 0
        //不能是同一路段
        && !(GN_point->path.source_index== point_pro->path.source_index && GN_point->path.target_index == point_pro->path.target_index))
    {
        double length_time = 0.0;
        if (path->first.type == 0) { length_time = agv_length / pro_path->first.car_v; }//AGV车长/速度
        else { length_time = fork_length/ pro_path->first.car_v; }//叉车车长/速度
        
        if ((GN_point->start_time < ((point_pro->end_time)/*+ length_time*/)
            && GN_point->start_time > point_pro->start_time)
            || (GN_point->end_time < ((point_pro->end_time) /*+ length_time*/)
                && GN_point->end_time > point_pro->start_time))
        {   
            auto GN_pointg = &((*path).second[0]);
            if (k != 0) {
                GN_pointg = &((*path).second[k - 1]);
            }
            if (GN_pointg->path.target_index == point_pro->path.target_index) {
                //不能在前车目标点等待
                for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//删除碰撞路段的目标点
                {
                    if (it->index == GN_pointg->index)
                    {
                        for (uint tt = 0; tt < it->link_edges.size(); tt++)
                        {
                            if (it->link_edges[tt].target_index == point_pro->path.target_index
                                )
                            {
                                it->link_edges[tt].state = false;
                            }
                        }
                    }
                }
                new_way=findPath((*GNs)[(*path).second[0].GN.index], (*GNs)[(*path).second.size()-1], GNs);
                pair<Car_config, pathList> temp;
                temp.first = (*path).first;
                temp.second = new_way;
                new_way = init_time_windows(&temp, GNs);

            }
            else {//等待
                double length_time = 0.0;
                if ((*path).first.type == 0) { length_time = agv_length / (*path).first.car_v; }
                else{ length_time = fork_length / (*path).first.car_v; }
                                      //前车走完后再出发                                   车长时间
                double set_time = fabs(point_pro->end_time - (*path).second[k].start_time)+ length_time;//计算后移时间

                cout<< "冲突集执行等待" << endl;
                for (uint m = k; m < (*path).second.size(); m++)//后续的点全部向后
                {
                    if (m == k) {
                        if ((*path).first.type == 0) {
                            double push_time = std::pow(((*path).first.car_v / agv_acc), 0.5) / (*path).second[m].path.leng;

                                (*path).second[m].start_time += set_time;
                                 set_time += push_time;
                                (*path).second[m].spend_time += push_time;
                                (*path).second[m].end_time += set_time;
                        }
                        else {
                            double push_time = std::pow(((*path).first.car_v / fork_acc), 0.5) / (*path).second[m].path.leng;

                            (*path).second[m].start_time += set_time;
                            set_time += push_time;
                            (*path).second[m].spend_time += push_time;
                            (*path).second[m].end_time += set_time;
                        }
                    }
                    else {
                        (*path).second[m].start_time += set_time; //后移时间窗 
                        (*path).second[m].end_time += set_time;
                    }
                }
                new_way =(*path).second;
            }
            //冲突集判断
        }
    }
    return new_way;
}
ASplanner::pathList ASplanner::Generator::init_time_windows(pair<Car_config,pathList> *path,vector<G_Node>* GNs)
{
    double time_cnt = 0;
    path_point previous;//上一个点
    for (auto& GN_point : (*path).second)
    {
        double cos = 1;
        if (GN_point.index != 0)
        {
            double x1 = GN_point.GN.coordinates.x - previous.GN.coordinates.x;
            double y1 = GN_point.GN.coordinates.y - previous.GN.coordinates.y;

            double x2 = ((*GNs)[GN_point.path.target_index].coordinates.x) - (GN_point.GN.coordinates.x);
            double y2 = ((*GNs)[GN_point.path.target_index].coordinates.y) - (GN_point.GN.coordinates.y);
            // cout << x1 << " " << y1 << endl;
            cos = (x1 * x2 + y1 * y2) / ((std::pow((pow(x1, 2) + pow(y1, 2)), 0.5) * std::pow((pow(x2, 2) + pow(y2, 2)), 0.5)));
            
        }

        if (GN_point.index == 0) {      //0的话，是第一个节点，计算启动时间
            
            if ((*path).first.type==0) 
            {
                double acc_leng =( agv_acc * pow(((*path).first.car_v / agv_acc), 2))/2;
                
                    //加速度时间  +  匀速时间
                GN_point.spend_time = (*path).first.car_v / agv_acc + (GN_point.path.leng - acc_leng) / (*path).first.car_v;
                
            }
            else {
                double acc_leng = (fork_acc * pow(((*path).first.car_v / fork_acc), 2)) / 2;// 加满速度长度
                 //加速度时间 +  匀速时间
                GN_point.spend_time = (*path).first.car_v / fork_acc + (GN_point.path.leng - acc_leng) / (*path).first.car_v;
             }
        }
        else if (GN_point.index == -1) {    //-1的话，代表没有节点
            GN_point.spend_time = 0;
        }
        
        else {                  //其他的话，代表正常路段的时间

            GN_point.spend_time = GN_point.path.leng / (*path).first.car_v+ (1 - cos) * wheel_time;//考虑转弯 
        }
        GN_point.start_time = time_cnt ;//起始时间
        time_cnt += GN_point.spend_time ;
        GN_point.end_time = time_cnt;
        previous = GN_point;
    }
    return (*path).second;
}
//同向冲突
ASplanner:: car_path ASplanner::Generator::colliding_conflict(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs)
{
    auto GN_point = &((*path).second[k]);
    auto point_pro = &((*pro_path).second[n]);

    if (GN_point->GN.index == point_pro->GN.index &&
        GN_point->path.target_index == point_pro->path.target_index
        //在此不考虑最后一节路，现实中应该不会发生吧
         )
    {   //后车跟前车
        //if ((*path).second.size() > 2 && k< (*path).second.size()-2)
        //{//小车i在此段路末尾执行等待了，优先级高的是否不用执行等待而撞上
        //    if ((*path).second[k].end_time < point_pro->end_time  //比前车早结束
        //        && (*path).second[k].end_time != (*path).second[k + 1].start_time//但是在这段路段尾执行等待了
        //        && pro_next->start_time < (*path).second[k + 1].start_time

        //        ) {
        //    }
        //}
        if (GN_point->start_time < point_pro->start_time
            && GN_point->end_time > point_pro->end_time && GN_point->end_time > point_pro->start_time
             ) 
             /* [ ]       我
            [        ]    前车*/
        {
            //找到分叉路，让优先级高的车先走
        }
         else if (GN_point->start_time < point_pro->end_time && GN_point->start_time > point_pro->start_time
             && GN_point->end_time < point_pro->end_time && GN_point->end_time > point_pro->start_time
             ) {
            //采取后车减速
            cout <<"减速执行等待。" << (*path).first.car_v << "m/s减速为" << pro_path->first.car_v << "m/s" << endl;
            (*path).first.car_v = pro_path->first.car_v;
            double moderate_time = (*path).second[k].start_time;
            for (uint m = k; m < (*path).second.size(); m++) {
                (*path).second[m].start_time = moderate_time;
                (*path).second[m].spend_time = (*path).second[m].path.leng / (*path).first.car_v;
                moderate_time += (*path).second[m].spend_time;
                (*path).second[m].end_time = moderate_time;
            }
        }
    }
    return (*path);
}

//节点冲突
ASplanner::pathList ASplanner::Generator::node_conflict(car_path* path,uint k ,car_path* pro_path,uint n ,vector<G_Node>* GNs)
{
    
    if ((*path).second[k].path.source_index == pro_path->second[n].path.target_index //起点为前车的目标点
        && (*path).second[k].path.target_index == pro_path->second[n].path.source_index
        && pro_path->second.size() > 2  //大于等于三
        && pro_path->second[n].index < ((*pro_path).second.size() - 2)
        &&path->second.size()>2
        &&path->second[k].index !=0  //不为第一个点，确保有后退空间
        ) //碰撞路段不是终点路段 且不为后车的起点路段
    {
        double length_time = 0.0;
        if (pro_path->first.type==0) { length_time = agv_length / pro_path->first.car_v; }//AGV车长/速度
        else { length_time = fork_length / pro_path->first.car_v; }//叉车车长/速度

        if (((*path).second[k].start_time < (*pro_path).second[n].end_time + length_time && (*path).second[k].start_time >(*pro_path).second[n].start_time)
            || ((*path).second[k].end_time < (*pro_path).second[n].end_time + length_time && (*path).second[k].end_time >(*pro_path).second[n].start_time))
        {
            auto pro_next = &(*pro_path).second[(*pro_path).second[n].index+1];
            auto GN_pointg = &(*path).second[k - 1];
            if (GN_pointg->GN.index != pro_next->path.target_index
                &&GN_pointg->path.target_index == pro_next->GN.index) {//碰撞路段后两车走不同路
                double t = (*pro_path).second[n].end_time - ((((*path).second[k].path.leng / (*pro_path).first.car_v) + fabs((*path).second[k].start_time +(*pro_path).second[n].start_time)) / 2);
                
                double set_time = fabs(pro_next->end_time - (*path).second[k].start_time) + t;//计算后移时间
                cout <<"节点冲突执行等待" << endl;
                for (uint m = k-1; m < (*path).second.size(); m++)//后续的点全部向后
                {
                    if (m == k-1) {
                        if ((*path).first.type == 0) {
                            double push_time = std::pow(((*path).first.car_v / agv_acc), 0.5) / (*path).second[m].path.leng;
                             (*path).second[m].start_time += set_time;
                            set_time += push_time;
                            (*path).second[m].spend_time += push_time;
                            (*path).second[m].end_time += set_time;
                        }
                        else {
                            double push_time = std::pow(((*path).first.car_v / fork_acc), 0.5) / (*path).second[m].path.leng;

                            (*path).second[m].start_time += set_time;
                            set_time += push_time;
                            (*path).second[m].spend_time += push_time;
                            (*path).second[m].end_time += set_time;
                        }
                    }
                    else {
                        (*path).second[m].start_time += set_time; //后移时间窗 
                        (*path).second[m].end_time += set_time;
                    }
                }
            }
        }
    }
    /*if ((*path).second[k].GN.index != point_pro->path.source_index
        && (*path).second[k].path.target_index == point_pro->path.target_index) {
        double del_time = fabs((*path).second[k].end_time - point_pro->end_time);
         const double* length = NULL;
        if (pro_car->type == 0) { length = &agv_length; }
        else { length = &fork_length; }
        if (del_time <(*length) /pro_car->car_v) {
            for (uint m = k; k < (*path).second.size(); k++)
            {
                if (m == k) {
                    (*path).second[k].end_time += (*length) / pro_car->car_v;
                    (*path).second[k].spend_time += (*length) / pro_car->car_v;
                }
                else {
                    (*path).second[m].end_time += (*length) / pro_car->car_v;
                    (*path).second[m].start_time += (*length) / pro_car->car_v;
                }
            }
        }
    }*/
    return (*path).second;
}
ASplanner::pathList ASplanner::Generator::station_is_vechel(uint k,uint pro_size,path_point* point_pro, pair<Car_config,pathList>* path, vector<G_Node>* GNs)
{
    pathList new_way = {};
    int size = path->second.size();
    if ((*path).second[k].path.target_index == point_pro->path.target_index && point_pro->index == pro_size
        &&point_pro->GN.name.last_id ==0
        )
    {
        //cout <<point_pro.GN.name.main_id<<"-" << point_pro.GN.name.sec_id <<"-"<< point_pro.GN.name.last_id << endl;
            if ((*path).second[k].end_time > point_pro->start_time)
        {
            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//删除碰撞路段的目标点
            {
                for (uint tt = 0; tt < it->link_edges.size(); tt++)
                {
                    if (it->link_edges[tt].target_index == point_pro->path.target_index)
                    {
                        it->link_edges[tt].state = false;
                    }
                }
            }
            uint start_index = (*path).second[0].GN.index;
            uint end_index = (*path).second[size - 1].GN.index;
            new_way = findPath((*GNs)[start_index], (*GNs)[end_index], GNs);
            pair<Car_config, pathList> temp;
            temp.second = new_way;
            temp.first =(*path).first ;
            new_way = init_time_windows(&temp, GNs);
            std::cout << "重新规划完成" << endl;
        }
    }
    return new_way;
}
