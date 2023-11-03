#include "ASplanner.h"
#include <algorithm>
#include <math.h>
#include <iostream>

using namespace std;
using namespace std::placeholders;
const double mini_distance = 1.0;
const double fork_length = 1.2;
const double agv_length = 0.8;
const double fork_acc =0.8;
const double agv_acc = 1.0;
const double de_time =2.0;   
const double wheel_time=1.0;
using namespace ASplanner;
bool  Vec2i::operator == (const Vec2i& coordinates_)
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

double ASplanner::Generator::conflict_check(vector<pair<Car_config,pathList>>* paths, vector<G_Node>* GNs)
{
    uint ff = 0;
    for (uint i = 0; i < (*paths).size(); i++) {    // i小车数量
        if ((*paths).size() == 10) { exit(1); }
        ff++;
        if (i == 0)
        {
            continue;
        }
        bool flag = true;
        uint cycle_count = 0;
        while (flag) {
            flag = false;
            cycle_count++;
            for (uint k = 0; k < (*paths)[i].second.size() && !flag; k++)//比对，待调整点。k:第i辆小车的路径节点位数
            {
                for (uint j = 0; j < i; j++)
                {
                    if ((*paths)[j].first.index == -1) {
                        continue;
                    }
                    if (flag)
                    {
                        break;
                    }
                    for (uint n = 0; (*paths)[j].first.index != -1&& n < (*paths)[j].second.size(); n++)//优先级更高的轨迹点
                    {
                        
                        bool is_mini_distance_return = mini_distance_between_vechels(&(*paths)[i], k, &(*paths)[j], n, GNs,paths, i, j);
                        if (is_mini_distance_return)
                        {
                            flag = true;
                            break;
                        }

                        if ((*paths)[j].first.index != -1) {
                            store_is_vechel(&(*paths)[i], k, &(*paths)[j], n, GNs, paths, i, j);
                        }

                       // 节点冲突
                        /*if ((*paths)[j].second.size() >2 ) {
                             node_conflict(&(*paths)[i],k,&(*paths)[j],n, GNs);
                        }*/

                        //同向冲突
                        if ((*paths)[j].first.index !=-1) {
                            bool is_collidiong_return = colliding_conflict(&(*paths)[i], k, &(*paths)[j], n, GNs, paths, i, j);
                        
                            if (is_collidiong_return)
                            {
                                flag = true;
                                break;
                            }
                        }
                        //冲突集判断及其策略
                        if ((*paths)[j].first.index != -1) {
                            bool is_collsion_return = collsion_collection(&(*paths)[i], k, &(*paths)[j], n, GNs,paths, i, j);
                            if (is_collsion_return)
                            {
                                flag = true;
                                break;
                            }
                        }
                        ////相向冲突
                        if ((*paths)[i].second.size() > 2 && (*paths)[j].first.index != -1){
                            bool is_conflict= opposing_conflict(&(*paths)[i], k, &(*paths)[j], n, GNs);
                            if (is_conflict) {
                                flag = true;
                                break;
                            }
                        }

                        if ((*paths)[j].first.index != -1) {
                            bool is_node_wait = node_check(&(*paths)[i], k, &(*paths)[j], n, GNs);
                            if (is_node_wait) {
                                flag = true;
                                break;
                            }
                        }
                    }
                }//车m
            }//车i的k
        }//while循环
        std::cout<<(*paths)[i].first.index <<" 车" << "while循环次数 ：" << cycle_count << endl;
        if (cycle_count == 10) { std::cout << "循环次数过多，自动退出程序" << endl; exit(1); }
        for (auto t = GNs->begin(); t != GNs->end(); t++)//恢复地图道路信息
        {
            for (auto edge : t->link_edges)
            {
                edge.state = true;
            }
        }
    }
    cout << "车数：" << ff << endl;
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
    /*if (time_path[time_path.size()-1].GN.index != target_.index)
    {
        cout << "寻路失败！" << endl;
        time_path = {};
    }*/
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

double  ASplanner::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

double  ASplanner::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

double  ASplanner::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
//相向冲突
bool ASplanner::Generator::opposing_conflict(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs)
{
    bool is_conflict=false;
    pathList new_way = {};
    auto point_pro = &(*pro_path).second[n];
    auto GN_point = &(*path).second[k];
    uint pro_size = pro_path->second.size();
    uint size = path->second.size();
    double pro_length=0.0;
    if (pro_path->first.type == 0) { pro_length = agv_length; }
    else { pro_length = fork_length; }

    if (GN_point->GN.index == point_pro->path.target_index
        && GN_point->path.target_index == point_pro->path.source_index){
        if (((GN_point->start_time < point_pro->end_time) && (GN_point->start_time > point_pro->start_time))
            || ((GN_point->end_time < point_pro->end_time) && (GN_point->end_time > point_pro->start_time))
            || ((GN_point->start_time < point_pro->start_time)&& (GN_point->end_time > point_pro->end_time))
            || ((GN_point->start_time > point_pro->start_time) && (GN_point->end_time < point_pro->end_time))) {
            bool is_solve = false;
            if (k > 0 && pro_size > 2 && point_pro->index < (pro_size - 2)
                && path->second[k - 1].GN.index != pro_path->second[n + 1].path.target_index) { //往后退一段试探可否躲避
                double wait_time = pro_path->second[n + 1].end_time + (pro_length / pro_path->first.car_v) - (path->second[k - 1].start_time);
                for (uint m = k - 1; m < size; m++) {
                    (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                }
                //向相冲突，后退一节等待
                cout << "相向冲突,后退一步" << endl;
                is_conflict = true;
                is_solve = true;
            }
       else if (k > 1 && pro_size > 3 && point_pro->index < (pro_size - 3)
                && path->second[k - 2].GN.index != pro_path->second[n + 2].path.target_index) {//回退两段路试试能不能通过等待躲过
                
                double wait_time = pro_path->second[n + 2].end_time + (pro_length / pro_path->first.car_v) - (path->second[k - 2].start_time);
                for (uint m = k - 2; m < size; m++) {
                    (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                } cout << "相向冲突,后退两步" << endl;
                is_conflict = true;
                is_solve = true;
            }
       else if (k > 2 && pro_size > 4 && point_pro->index < (pro_size - 4)
                && path->second[k - 3].GN.index != pro_path->second[n + 3].path.target_index) {
                
                double wait_time = pro_path->second[n + 3].end_time + (pro_length / pro_path->first.car_v) - (path->second[k - 3].start_time);
                for (uint m = k - 3; m < size; m++) {
                    (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                } cout << "相向冲突,后退三步" << endl;
                is_conflict = true;
                is_solve = true;
            }
            else {      //需要重新规划  绕路
                uint end_index = ((*path).second[size - 1].GN.index);  //最后一个元素索引号地址
                uint start_index = ((*path).second[0].GN.index);

                if (((path->first.car_v > pro_path->first.car_v) || (path->first.car_v == pro_path->first.car_v)) && k > 0){
                    
                    auto GN_pointg = &(*path).second[k - 1];
                    replanning_path(path,GN_pointg,GNs);
                    
                    std::cout << "相向冲突规划完成" << endl;
                    is_conflict = true;
                    is_solve = true;
                }

                else if ((path->first.car_v < pro_path->first.car_v)) {

                    auto GN_pointg = &(*path).second[0];
                    if (k > 1) { GN_pointg = &(*path).second[k - 2]; }

                    replanning_path(path, GN_pointg, GNs);

                    std::cout << "相向冲突规划完成" << endl;
                    is_conflict = true;
                    is_solve = true;
                }
            }
            if (!is_conflict) {
                cout << "未解决相向冲突" << endl;
                exit(1);
            }
        }
    }
    return is_conflict;
}
//冲突集
bool ASplanner::Generator::collsion_collection(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs,vector<pair<Car_config, pathList>>* paths, uint i, uint j)
{
     bool is_return=false;
     auto point_pro = &(*pro_path).second[n];
     auto GN_point = &(*path).second[k];
     uint pro_size = pro_path->second.size();
     uint size = path->second.size();
     double pro_length=0.0;
     if (path->first.type == 0) { pro_length = agv_length; }//AGV车长
     else { pro_length= fork_length ; }//叉车车长

    if (GN_point->path.collsion_id == point_pro->path.collsion_id &&GN_point->path.collsion_id != 0    //冲突集号相同
        && !((GN_point->GN.index == point_pro->GN.index && GN_point->path.target_index == point_pro->path.target_index)
        || (GN_point->GN.index == point_pro->path.target_index && GN_point->path.target_index == point_pro->GN.index))){//不能是同一路段
    
        if ((GN_point->start_time < ((point_pro->end_time)/*+ length_time*/)
            && GN_point->start_time > point_pro->start_time)
            || (GN_point->end_time < ((point_pro->end_time) /*+ length_time*/)
                && GN_point->end_time > point_pro->start_time)
            ||((GN_point->start_time < ((point_pro->start_time)/*+ length_time*/)
                && GN_point->end_time > point_pro->end_time)
            ||((GN_point->start_time > ((point_pro->start_time)/*+ length_time*/)
                && GN_point->end_time <  point_pro->end_time)) )){ //存在时间交叉重叠 
            
            if (GN_point->path.target_index == point_pro->GN.index) {                 //情况0、后车终点为前车起点
                double wait_time = point_pro->end_time + ((pro_length) / pro_path->first.car_v) - (GN_point->start_time);
                for (uint m = k; m < size; m++) {
                    //后续时间窗延后
                   (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                }
                cout << "后车终点为前车起点，时间窗后延" << endl;
            }
            else if (GN_point->GN.index == point_pro->path.target_index) {          //情况1、前车目标点是后车起点
                if (k > 0 && pro_size > 2 &&(point_pro->index) < (pro_size - 2)//长度足够
                    && path->second[k - 1].GN.index != pro_path->second[n + 1].path.target_index) {//回退一步，试探。
                    double wait_time=pro_path->second[n + 1].end_time + ((pro_length) / pro_path->first.car_v) - path->second[k - 1].start_time;
                    for (uint m = k - 1; m < size; m++) {                 //后续时间窗延后
                        (*path).second[m].start_time += wait_time;
                        (*path).second[m].end_time += wait_time;
                        }
                    is_return = true;
                }//试探结束

            else if (k > 1 && pro_size > 3 && (point_pro->index) < (pro_size - 3)
                    && path->second[k - 2].GN.index != pro_path->second[n + 2].path.target_index ) {//回退两步，试探
                    double wait_time = pro_path->second[n + 2].end_time + (pro_length) / pro_path->first.car_v - path->second[k - 2].start_time;
                        for (uint m = k - 2; m < size; m++) {
                            (*path).second[m].start_time += wait_time;
                            (*path).second[m].end_time += wait_time;
                        }
                        is_return = true;
                 }//试探结束

                else {   //试探完了，没办法了。重新规划
                    auto GN_pointg = &(path->second[0]);
                    if (k != 0) { GN_pointg = &(path->second[k - 1]); }
                    replanning_path(path, GN_pointg, GNs);
                    is_return = true;
                   /* pair<Car_config, pathList>temp = (*paths)[j];
                    if (i == ((*paths).size() - 1)) {
                        (*paths).push_back(temp);
                    }
                    else { (*paths).insert((*paths).begin() + i + 1, temp); }
                    (*paths)[j].first.index = -1;
                    temp.second.clear();
                    (*paths)[j].second.clear();*/
                }
            }

            else if(GN_point->GN.index == point_pro->GN.index){    //情况2、起点相同，终点不同。分为两种情况
                bool is_solve = false;
                if (GN_point->start_time < point_pro->start_time) {//优先级低的车先到
                    for (uint ii = 1; ii < (k +1)&& ii < (n+1); ii++)//取最小值
                    {
                        if ((*path).second[k - ii ].GN.index != (*pro_path).second[n - ii ].GN.index
                            && (*path).second[k - ii ].path.target_index == (*pro_path).second[n - ii ].path.target_index)//找分岔路口
                        {
                            double length_time = 0.0;
                            if (pro_path->first.type == 0) { length_time = agv_length / pro_path->first.car_v; }
                            else { length_time = fork_length / pro_path->first.car_v; }

                            double wait_time = fabs(((*path).second[k - ii].start_time) - ((*pro_path).second[n - ii].end_time)) + length_time;
                            for (uint u = (k - ii ); u < ((*path).second.size()); u++)//后移时间窗
                            {
                                (*path).second[u].start_time += wait_time;
                                (*path).second[u].end_time += wait_time;
                            }
                            is_return = true;
                            is_solve=true;
                            break;
                        }
                    }
                //  如果没有找到分岔路口:解决方案：.....
                    if (!is_solve) {
                        pair<Car_config, pathList>temp = (*paths)[j];
                        if (i == ((*paths).size() - 1)) {
                            (*paths).push_back(temp);
                        }
                        else { (*paths).insert((*paths).begin() + i + 1, temp); }
                        (*paths)[j].first.index = -1;
                        temp.second.clear();
                        (*paths)[j].second.clear();
                    }
               
                }
                else {//优先级高的车先到
                    if (GN_point->end_time < (mini_distance + (pro_length) / pro_path->first.car_v) + GN_point->end_time && k>0) {
                        double wait_time = (mini_distance + (pro_length) / pro_path->first.car_v) + GN_point->end_time - GN_point->end_time;
                        for (uint m = k-1; m < size; m++) {
                            (*path).second[m].start_time += wait_time;
                            (*path).second[m].end_time += wait_time;
                        }
                    }
                }
            }

       else if(GN_point->GN.index != point_pro->GN.index                
                &&GN_point->path.target_index==point_pro->path.target_index){//情况3、起点不同、终点相同。执行等待
                
                double set_time = point_pro->end_time - GN_point->start_time +((pro_length) + mini_distance) / pro_path->first.car_v;//计算后移时间

                cout<< "冲突集执行等待" << endl;
                for (uint m = k; m < size; m++)//后续的点全部向后
                {  (*path).second[m].start_time += set_time; //后移时间窗 
                   (*path).second[m].end_time += set_time;
                }
            }
            //冲突集判断
        }
    }
    return is_return;
}
//时间窗初始化
void ASplanner::Generator::init_time_windows(double time_cnt,pair<Car_config,pathList> *path,vector<G_Node>* GNs)
{
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
            
            if ((*path).first.type==0){
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
}

//同向冲突,主要解决优先级低的车因等待被超问题
bool ASplanner::Generator::colliding_conflict(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs, vector<pair<Car_config, pathList>>* paths,uint i,uint j)
{   
    bool is_return = false;
    car_path new_path = {};
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    uint pro_size = pro_path->second.size();
    uint size = path->second.size();
    if(GN_point->GN.index == point_pro->GN.index 
        &&GN_point->path.target_index == point_pro->path.target_index){     //同一段路径
        
        if (k < (size - 2) && n < (pro_size - 2)                           //保证此段路不为最后一段路
            && GN_point->start_time < point_pro->start_time                    //此节点先到
            && (*path).second[k + 1].start_time >(*pro_path).second[n + 1].start_time) {    //但是下一个节点被超了
            bool is_solve = false;     //是否有分叉路
            for (uint ii = 1; ii < (k+1) && ii < n; ii++)//取最小值
            {
                if ((*path).second[k - ii].GN.index != (*pro_path).second[n - ii].GN.index
                    && (*path).second[k - ii].path.target_index == (*pro_path).second[n - ii].path.target_index) {

                    double length_time = 0.0;
                    if (pro_path->first.type == 0) { length_time = agv_length / pro_path->first.car_v; }
                    else { length_time = fork_length / pro_path->first.car_v; }
                    double wait_time = ((*pro_path).second[n - ii].end_time) - ((*path).second[k - ii].start_time) + length_time;
                    for (uint u = (k - ii); u < ((*path).second.size()); u++)//后移时间窗
                    {
                        (*path).second[u].start_time += wait_time;
                        (*path).second[u].end_time += wait_time;
                    }
                    is_solve = true;
                    is_return = true;
                    break;
                }
            }

            if (!is_solve) {              //审核员审核是否解决
                pair<Car_config, pathList>temp = (*paths)[j];
                if (i == ((*paths).size() - 1)) {
                    (*paths).push_back(temp);
                }
                else { (*paths).insert((*paths).begin() + i + 1, temp); }
                (*paths)[j].first.index = -1;
                temp.second.clear();
                (*paths)[j].second.clear();
            }
        
        }
       
   else if (k < (size - 2) && n < (pro_size - 2)
        &&GN_point->start_time<point_pro->start_time
            && (*path).second[k+1].start_time < (*pro_path).second[n+1].start_time) {
            //采取后车减速
            cout << "减速执行等待。" << (*path).first.car_v << "m/s减速为" << pro_path->first.car_v << "m/s" << endl;
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
    return is_return;
}

//节点冲突
void ASplanner::Generator::node_conflict(car_path* path,uint k ,car_path* pro_path,uint n ,vector<G_Node>* GNs)
{
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    const double* pro_length;
    if (pro_path->first.type == 0) { pro_length = &agv_length; }//AGV车长/速度
    else { pro_length = &fork_length; }//叉车车长/速度

    if (GN_point->GN.index != point_pro->path.source_index // 两条路起点不一样
        && (*path).second[k].path.target_index == point_pro->path.target_index //目标点相同
        && GN_point->path.collsion_id != point_pro->path.collsion_id ){
        double delt_time = fabs(GN_point->end_time - point_pro->end_time);
        if (delt_time < ((mini_distance + *pro_length) / pro_path->first.car_v)) {   //时间差不够大
            double wait_time = point_pro->end_time + (mini_distance + (*pro_length)) / pro_path->first.car_v - (GN_point->spend_time) - (GN_point->start_time);
            for (uint m = k; m < (*path).second.size(); m++) {
                
                (*path).second[m].start_time += wait_time;
                (*path).second[m].end_time += wait_time;
            }
        }
    }
}
//最小车距
bool ASplanner::Generator::mini_distance_between_vechels(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs,vector<pair<Car_config, pathList>>* paths, uint i, uint j)
{
    bool is_return = false;
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    uint size = path->second.size();
    uint pro_size = pro_path->second.size();
    const double* pro_length;
    const double* length;
    if (pro_path->first.type == 0) { pro_length = &agv_length; }
    else { pro_length = &fork_length; }
    if (path->first.type == 0) { length = &agv_length; }
    else { length = &fork_length; }
    const double mini_time = (mini_distance + (*pro_length)) / (*pro_path).first.car_v;

    if (((path->first.car_v > pro_path->first.car_v) || (path->first.car_v == pro_path->first.car_v))   //车速大于或等于优先级高的车
        &&GN_point->GN.index==point_pro->GN.index && GN_point->path.target_index== point_pro->path.target_index//同一段路
        &&GN_point->index <(size-2) && point_pro->index<(pro_size-2)) {     //不为最后一段路
        
        if ((GN_point->start_time > point_pro->start_time)         //优先级高的车在前
            && (GN_point->start_time - point_pro->start_time) < mini_time) {  //但是时间差不够大
            (*path).first.car_v = pro_path->first.car_v;      //减速
            
            double cnt_time = (*path).second[k].start_time;
            for (uint m = k; m < size;m++) {

                (*path).second[m].start_time = cnt_time;
                (*path).second[m].spend_time= (*path).second[m].path.leng / path->first.car_v;
                cnt_time += (*path).second[m].spend_time;
                (*path).second[m].end_time = cnt_time;
            }    

            if ((GN_point->spend_time > mini_time)||(GN_point->spend_time == mini_time)) {        //一段路时间够不够
                for (uint m = k; m < size; m++) {
                    (*path).second[m].start_time += mini_time;
                    (*path).second[m].end_time += mini_time;
                }
            }

       else if (point_pro->spend_time < mini_time && k<(size-2)
                &&((*path).second[k+1].spend_time+GN_point->spend_time)>mini_time) { //两段路时间够吗？
                
                double wait_time=mini_time/*最小时间*/ - GN_point->spend_time + point_pro->end_time-GN_point->start_time;
                    
                for (uint m = k-1; m < size; m++) {
                    (*path).second[m].start_time += mini_time;
                    (*path).second[m].end_time += mini_time;     
                }
            }

       else if (point_pro->spend_time < mini_time&& k<(size-3)&&n<(pro_size-3)
           && ((*path).second[k + 1].spend_time + GN_point->spend_time) > mini_time) { //三段路时间够吗？

                double wait_time =mini_time - GN_point->spend_time - (*path).second[k+1].spend_time + point_pro->end_time - GN_point->start_time;

                for (uint m = k-2; m < size; m++) {
                    (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                }
            }
            else {
            
                for (uint m = k; m < size; m++) {
                    (*path).second[m].start_time += mini_time;
                    (*path).second[m].end_time += mini_time;
                }
            }
        }
                                                                       
       else if(( point_pro->start_time > GN_point->start_time)   //优先级低的在前
           && (point_pro->start_time - GN_point->start_time) < mini_time){      //优先级高的车在后

            bool is_solve = false;
            for (uint ii = 1; ii < (k+1) && ii < (n+1); ii++)//取最小值
            {
                if ((*path).second[k - ii].GN.index != (*pro_path).second[n - ii].GN.index
                    && (*path).second[k - ii].path.target_index == (*pro_path).second[n - ii].path.target_index) {

                    double wait_time = (*pro_path).second[n - ii].end_time + (mini_time - (*path).second[k - ii].spend_time) - (*path).second[k - ii].start_time;
                    for (uint u = (k - ii); u < ((*path).second.size()); u++)//后移时间窗
                    {
                        (*path).second[u].start_time += wait_time;
                        (*path).second[u].end_time += wait_time;
                    }
                    is_solve = true;
                    is_return = true;
                    break;
                }
            }

            if (!is_solve) { //调整优先级
                pair<Car_config, pathList>temp = (*paths)[j];
                if (i == ((*paths).size() - 1)) {
                    (*paths).push_back(temp);
                }
                else { (*paths).insert((*paths).begin() + i +1, temp); }
                (*paths)[j].first.index = -1;
                temp.second.clear();
                (*paths)[j].second.clear();
               std::cout << "优先级高的车在后" << endl;
            }
        }
    }

    else if ((path->first.car_v < pro_path->first.car_v)            //车速小于优先级高的车
        && GN_point->GN.index == point_pro->GN.index && GN_point->path.target_index == point_pro->path.target_index) {//同一段路

            if (GN_point->start_time - point_pro->start_time > 0//优先级高的车在前
                && GN_point->start_time - point_pro->start_time < mini_time) {//差的不多
                double wait_time = mini_time - GN_point->start_time + point_pro->start_time;
                for (uint m = k; m < size; m++) {
                (*path).second[m].start_time += wait_time;
                (*path).second[m].end_time += wait_time;
                }
            }

            else if (GN_point->start_time < point_pro->start_time ){ //比优先级高的车先进来。时空回溯
                    //时空回溯
                bool is_solve = false;
                for (uint ii = 1; ii < (k+1) && ii < (n+1); ii++)//取最小值
                {
                    if ((*path).second[k - ii].GN.index != (*pro_path).second[n - ii].GN.index
                        && (*path).second[k - ii].path.target_index == (*pro_path).second[n - ii].path.target_index) {

                        double wait_time = (*pro_path).second[n - ii].end_time +(mini_time - (*path).second[k - ii].spend_time)- (*path).second[k - ii].start_time;
                        for (uint u = (k - ii); u < ((*path).second.size()); u++)//后移时间窗
                        {
                            (*path).second[u].start_time += wait_time;
                            (*path).second[u].end_time += wait_time;
                        }
                        is_solve = true;
                        is_return = true;
                        break;
                    }
                }
                if (!is_solve) {
                    pair<Car_config, pathList>temp = (*paths)[j];
                    if (i == ((*paths).size() - 1)) {
                        (*paths).push_back(temp);
                    }
                    else { (*paths).insert((*paths).begin() + i+1, temp); }
                    (*paths)[j].first.index = -1;
                    temp.second.clear();
                    (*paths)[j].second.clear();
                    cout << "低优先级的车在前，调整优先级！" << endl;
                }
            }
         }

    return is_return;
}

bool ASplanner::Generator::node_check(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs)
{
    bool is_return = false;
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    uint size = path->second.size();
    uint pro_size = pro_path->second.size();
    if (n < (pro_size - 2) && k != 0) {
        auto GN_pointg = &(*path).second[k - 1];
        auto pro_next = &(*pro_path).second[n + 1];
        uint size = path->second.size();
        uint pro_size = pro_path->second.size();
        if (GN_point->GN.index == pro_next->path.target_index && GN_point->path.target_index == pro_next->GN.index
            //&& GN_pointg->GN.index == point_pro->path.target_index && GN_pointg->GN.index == point_pro->GN.index
            && point_pro->end_time != pro_next->start_time  //前车在该节点有等待
            && GN_point->end_time > point_pro->end_time && GN_point->end_time < pro_next->start_time) { //在前车等待时间路过此节点
            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//删除碰撞路段的目标点
            {
                if (it->index == GN_point->GN.index)
                {
                    for (uint tt = 0; tt < it->link_edges.size(); tt++)
                    {
                        if (it->link_edges[tt].target_index == GN_point->path.target_index) {
                            it->link_edges[tt].state = false;
                        }
                    }
                }
            }
           //重新规划
            is_return = true;
        }
    }
    return is_return;
}

//库区有车子
void ASplanner::Generator::store_is_vechel(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs, vector<pair<Car_config, pathList>>* paths, uint i, uint j) {
    bool is_return = false;
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    uint size = path->second.size();
    uint pro_size = pro_path->second.size();
    const double* pro_length;
    if (pro_path->first.type == 0) { pro_length = &agv_length; }
    else { pro_length = &fork_length; }
    if (k < (size - 1)&&(*GNs)[(*path).second[k + 1].path.target_index].name.last_id !=0) {  //该车需要进入断头路
        path_point* GN_out= nullptr;//初始化
        for (uint count = k; count<(*path).second.size();count++) {
            if ((*path).second[count].GN.name.last_id == 0) {
                GN_out = &(*path).second[count];
                 break;
            }
        }
         path_point* pro_out=nullptr;
        if (n < (pro_size - 1)&& (*GNs)[(*pro_path).second[n+ 1].path.target_index].name.last_id != 0 //优先级高的车要进入断头路
            &&(*GNs)[(*pro_path).second[n + 1].path.target_index].name.sec_id == (*GNs)[(*path).second[k + 1].path.target_index].name.sec_id ) {
            for (uint count = n; count < (*pro_path).second.size(); count++) {
                if ((*pro_path).second[count].GN.name.last_id == 0) {
                    pro_out=&(*pro_path).second[count];
                    break;
                }
            }
            if (GN_point->end_time < point_pro->end_time && GN_out->end_time > point_pro->start_time) {//优先级低的车先进入断头路
               //改变优先级
                pair<Car_config, pathList>temp = (*paths)[j];
                if (i == ((*paths).size() - 1)) {
                    (*paths).push_back(temp);
                }
                else { (*paths).insert((*paths).begin() + i + 1, temp); }
                (*paths)[j].first.index = -1;
                temp.second.clear();
                (*paths)[j].second.clear();

            }
            else if (GN_point->end_time > point_pro->end_time && GN_out->end_time < point_pro->start_time) {          //优先级高的车先进入断头路
                //判断出站方向,
                if (pro_out->path.target_index == GN_point->GN.index) {   //冲突
                    replanning_path(path, GN_point, GNs);
                    
                }

                else if (pro_out->path.target_index != GN_point->index) {//最简单的情况
                    double wait_time=pro_out->end_time+(*pro_length)/(pro_path->first.car_v)- (*path).second[k].start_time;
                    for (uint m = k; k < size;k++) {
                        (*path).second[m].start_time += wait_time;
                        (*path).second[m].end_time += wait_time;
                    }
                }
            }
        }
    }
}

void ASplanner::Generator::replanning_path(car_path* path,path_point* GN_point, vector<G_Node>* GNs) {
    
    for (auto it = (*GNs).begin(); it != (*GNs).end(); it++) {
        if ((*it).index == GN_point->GN.index) {
            for (auto tt = (*it).link_edges.begin(); tt != (*it).link_edges.end(); tt++) {
                if ((*tt).target_index == GN_point->path.target_index) {
                    (*tt).state = false;
                }
            }
        }
    }
    int* new_satrt_point = nullptr;  //地图重新规划时所需要的起点与终点的索引号
    int* new_end_point = nullptr;
    uint c = 0;
    int* start_point_index = nullptr;   //该段路在路径容器中索引号
    int* target_point_index = nullptr;
    for (c; c < (path->first.middle_point.size() - 1); c++) {
        if ((path->first.middle_point[c].first < GN_point->index || path->first.middle_point[c].first == GN_point->index)
            && GN_point->index < path->first.middle_point[c + 1].first) {
            new_satrt_point = &(*path).first.middle_point[c].second;
            new_end_point = &(*path).first.middle_point[c + 1].second;
            break;
        }
    }

    bool is_end_path = false;
    if (c +1== (path->first.middle_point.size()-1)) {  //若为最后一段路
        is_end_path = true;
    }

    start_point_index = &(path->first.middle_point[c].first);
    target_point_index = &(path->first.middle_point[c + 1].first);

    //删除老的路段
    double start_time = path->second[*start_point_index].start_time;
   (*path).second.erase((*path).second.begin() + (*start_point_index), (*path).second.begin() + (*target_point_index));

    pathList new_way = findPath((*GNs)[*new_satrt_point], (*GNs)[*new_end_point], GNs);
    car_path temp_path((*path).first, new_way);

    //时间窗初始化

    init_time_windows(start_time, &(temp_path), GNs);

    if (is_end_path) {
        (*path).second.insert(path->second.begin() + *start_point_index, temp_path.second.begin(), temp_path.second.end());
    }
    else {
        double del_time =temp_path.second.back().end_time -(*path).second[*start_point_index].start_time;

        (*path).second.insert(path->second.begin() + *start_point_index, temp_path.second.begin(), temp_path.second.end());

        for (uint m =*start_point_index+temp_path.second.size() ; m < path->second.size(); m++) {
            (*path).second[m].start_time += del_time;
            (*path).second[m].end_time += del_time;
        }
    }
    
     uint i = *start_point_index;
     //更新各个点的索引号
     for (uint m = i; m < path->second.size();m++) {
         if (m == path->second.size() - 1) { (*path).second[m].index = -1; }
         else {
             (*path).second[m].index = m;
         }
     }

     //更新各段路的起始点信息
     uint del_size = temp_path.second.size() - (*target_point_index - *start_point_index+1);
     for (uint m = c; c < path->first.middle_point.size() - 1;c++) {
         (*path).first.middle_point[c+1].first == (*path).first.middle_point[c].first + del_size;
     }
    
}
//ASplanner::pathList ASplanner::Generator::station_is_vechel(uint k,uint pro_size,path_point* point_pro, pair<Car_config,pathList>* path, vector<G_Node>* GNs)
//{
//    pathList new_way = {};
//    int size = path->second.size();
//    if ((*path).second[k].path.target_index == point_pro->path.target_index && point_pro->index == pro_size
//        &&point_pro->GN.name.last_id ==0
//        )
//    {
//        //cout <<point_pro.GN.name.main_id<<"-" << point_pro.GN.name.sec_id <<"-"<< point_pro.GN.name.last_id << endl;
//            if ((*path).second[k].end_time > point_pro->start_time)
//        {
//            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//删除碰撞路段的目标点
//            {
//                for (uint tt = 0; tt < it->link_edges.size(); tt++)
//                {
//                    if (it->link_edges[tt].target_index == point_pro->path.target_index)
//                    {
//                        it->link_edges[tt].state = false;
//                    }
//                }
//            }
//            uint start_index = (*path).second[0].GN.index;
//            uint end_index = (*path).second[size - 1].GN.index;
//            new_way = findPath((*GNs)[start_index], (*GNs)[end_index], GNs);
//            pair<Car_config, pathList> temp;
//            temp.second = new_way;
//            temp.first =(*path).first ;
//            new_way = init_time_windows(&temp, GNs);
//            std::cout << "重新规划完成" << endl;
//        }
//    }
//    return new_way;
//}
