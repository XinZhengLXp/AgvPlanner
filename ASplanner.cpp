#include "ASplanner.h"
#include <algorithm>
#include <math.h>
#include "tinyxml2.h"
#include <iostream>
using namespace tinyxml2;
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
    const uint car_count=paths->size();
    uint result = car_count;
    for (uint g = 1; g < car_count;++g) {
        result *= g;
    }
    uint ff = 0;
    for (uint i = 0; i < (*paths).size(); i++) {    // i小车数量
        if ((*paths).size() >result) {
            cout << "优先级改变次数，次数过多" << endl;
            exit(1); }
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
                        //
                        //// 节点冲突
                        if ((*paths)[j].second.size() > 2) {
                            node_conflict(&(*paths)[i], k, &(*paths)[j], n, GNs);
                        }

                        if ((*paths)[j].first.index != -1) {
                            bool is_node_wait = node_check(&(*paths)[i], k, &(*paths)[j], n, GNs);
                            if (is_node_wait) {
                                flag = true;
                                break;
                            }
                        }
                        ////
                        if ((*paths)[j].first.index != -1) {
                            bool is_collidiong_return = colliding_conflict(&(*paths)[i], k, &(*paths)[j], n, GNs, paths, i, j);

                            if (is_collidiong_return)
                            {
                                flag = true;
                                break;
                            }
                        }
                        //
                        if ((*paths)[j].first.index != -1&&(*paths)[j].second.size()!=0) {
                           
                            bool is_store_retrn = store_is_vechel(&(*paths)[i], k, &(*paths)[j], n, GNs, paths, i, j);
                            if (is_store_retrn)
                            {
                                flag = true;
                                break;
                            }
                        }

                        ////相向冲突
                        if ((*paths)[i].second.size() > 2 && (*paths)[j].first.index != -1) {
                            bool is_conflict = opposing_conflict(&(*paths)[i], k, &(*paths)[j], n, GNs, paths, i, j);
                            if (is_conflict) {
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
                        //cout << "检测一遍" << endl;
                       
                    }
                }//车m
            }//车i的k
        }//while循环
        std::cout<<(*paths)[i].first.index <<" 车" << "while循环次数 ：" << cycle_count << endl;
        //if (cycle_count == 10) { std::cout << "优先级变换次数过多，自动退出程序" << endl; exit(1); }
        for (auto t = GNs->begin(); t != GNs->end(); t++)//恢复地图道路信息
        {
            for (auto edge : t->link_edges)
            {
                edge.state = true;
            }
        }
    }
    cout << "优先级改变次数" << ff -car_count<< endl;
    
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
    G_NodeList path;   //G_Node
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
                /*else { cout << "A*路径不匹配" << endl; }*/
            }
        }
        time_path.push_back(temp_t);    //将路径的的各个站点压入time_path栈中
    }

    releaseNodes(openSet);  //释放栈空间
    releaseNodes(closedSet);//释放栈空间
   /* if (time_path[time_path.size()-1].GN.index != target_.index)
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
void ASplanner::Generator::delay_func(pathList* path,uint k,uint size,double *wait_time) {
    for (uint m = k; m < size;m++) {
        
        (*path)[m].start_time += *wait_time;
        (*path)[m].end_time += *wait_time;
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
bool ASplanner::Generator::opposing_conflict(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs, vector<pair<Car_config, pathList>>* paths, uint i, uint j)
{
    bool is_return=false;
    pathList new_way = {};
    auto point_pro = &(*pro_path).second[n];
    auto GN_point = &(*path).second[k];
    uint pro_size = pro_path->second.size();
    uint size = path->second.size();
    double pro_length=0.0;
    if (pro_path->first.type == 0) { pro_length = agv_length; }
    else { pro_length = fork_length; }

    if (GN_point->GN.index == point_pro->path.target_index
        &&GN_point->GN.name.last_id ==0&&point_pro->GN.name.last_id==0
        && GN_point->path.target_index == point_pro->path.source_index){
        if (((GN_point->start_time < point_pro->end_time) && (GN_point->start_time > point_pro->start_time))
            || ((GN_point->end_time < point_pro->end_time) && (GN_point->end_time > point_pro->start_time))
            ||(GN_point->end_time == point_pro->end_time)
            || ((GN_point->start_time <= point_pro->start_time)&& (GN_point->end_time >= point_pro->end_time))
            || ((GN_point->start_time >= point_pro->start_time) && (GN_point->end_time <= point_pro->end_time))) {
            bool is_solve = false;
            is_solve = backtrack(path,k, pro_path,n, GNs, pro_length);
            if (is_solve) { is_return = true; }
            /*if (path->second[0].GN.link_edges.size()==2&&(!is_solve)) {
                replanning_path(path,&(*path).second[0],GNs);
                is_solve = true;
                is_return = true;
            }*/

            if (!is_solve) {     //改变优先级
                pair<Car_config, pathList> temp = (*paths)[j];
                if (i == ((*paths).size() - 1)) {
                    (*paths).push_back(temp);
                }
                else { (*paths).insert((*paths).begin() + i + 1, temp); }
                (*paths)[j].first.index = -1;

                (*paths)[j].second.clear();
                (*paths)[j].second.shrink_to_fit();
                cout << "冲突集问题，调整优先级！" << endl;
            }
        }
    }
    return is_return;
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
    
        if (((GN_point->start_time < point_pro->end_time) && (GN_point->start_time > point_pro->start_time))
            || ((GN_point->end_time < point_pro->end_time) && (GN_point->end_time > point_pro->start_time))
            ||(GN_point->end_time == point_pro->end_time)
            ||((GN_point->start_time <= point_pro->start_time)&& (GN_point->end_time >= point_pro->end_time))
            ||((GN_point->start_time > point_pro->start_time)&& (GN_point->end_time <  point_pro->end_time))){ //存在时间交叉重叠 
            
            if (GN_point->path.target_index == point_pro->GN.index) {                 //情况0、后车终点为前车起点
                double* wait_time = new double(point_pro->end_time + ((pro_length) / pro_path->first.car_v) - (GN_point->start_time));//计算等待时间
                delay_func(&(*path).second, k, size, wait_time);//等待函数
                delete wait_time;
                cout << "后车终点为前车起点，时间窗后延" << endl;
            }
            else if (GN_point->GN.index == point_pro->path.target_index) {          //情况1、前车目标点是后车起点
               bool is_ok=backtrack(path,k,pro_path,n,GNs,pro_length);
               if (!is_ok) {
                   //改变优先级
                   pair<Car_config, pathList> temp = (*paths)[j];
                   if (i == ((*paths).size() - 1)) {
                       (*paths).push_back(temp);
                   }
                   else { (*paths).insert((*paths).begin() + i + 1, temp); }
                   (*paths)[j].first.index = -1;

                   (*paths)[j].second.clear();
                   (*paths)[j].second.shrink_to_fit();
               }
            }
            else if(GN_point->GN.index == point_pro->GN.index){    //情况2、起点相同，终点不同。分为两种情况
                bool is_solve = false;
                if (GN_point->start_time < point_pro->start_time) {//优先级低的车先到
                    for (uint ii = 1; ii < (k +1)&& ii < (n+1); ii++)//for循环从后往前回溯到合适的路
                    {
                        if ((*path).second[k - ii ].GN.index != (*pro_path).second[n - ii ].GN.index
                            && (*path).second[k - ii ].path.target_index == (*pro_path).second[n - ii ].path.target_index)//找分岔路口
                        {
                            double length_time = 0.0;
                            if (pro_path->first.type == 0) { length_time = agv_length / pro_path->first.car_v; }
                            else { length_time = fork_length / pro_path->first.car_v; }

                            double *wait_time =new double( fabs(((*path).second[k - ii].start_time) - ((*pro_path).second[n - ii].end_time)) + length_time);
                            delay_func(&(*path).second, k - ii, (*path).second.size(), wait_time);//后续时间窗延后
                            delete wait_time;
                            is_return = true;
                            is_solve=true;
                            break;
                        }
                    }
                //  如果没有找到分岔路口:解决方案：.....
                    if (!is_solve) {
                        pair<Car_config, pathList> temp =(*paths)[j];
                        if (i == ((*paths).size() - 1)) {
                            (*paths).push_back(temp);
                        }
                        else { (*paths).insert((*paths).begin() + i + 1, temp); }
                        (*paths)[j].first.index = -1;
                        
                        (*paths)[j].second.clear();
                        (*paths)[j].second.shrink_to_fit();
                        cout << "冲突集问题，调整优先级！" << endl;
                    }
               
                }
                else {//优先级高的车先到
                    if (GN_point->end_time < (mini_distance + (pro_length) / pro_path->first.car_v) + GN_point->end_time && k>0) {
                        double* wait_time = new double((mini_distance + (pro_length) / pro_path->first.car_v) + GN_point->end_time - GN_point->end_time);
                        
                        delay_func(&(*path).second, k - 1,size, wait_time);//后续时间窗延后
                        delete wait_time;
                    }
                }
            }

       else if(GN_point->GN.index != point_pro->GN.index                
                &&GN_point->path.target_index==point_pro->path.target_index){//情况3、起点不同、终点相同。执行等待
                
                double *set_time =new double( point_pro->end_time - GN_point->start_time +((pro_length) + mini_distance) / pro_path->first.car_v);//计算后移时间

                cout<< "冲突集执行等待" << endl;
                delay_func(&(*path).second, k, size, set_time);//后续时间窗延后
                delete set_time;
            }
            //冲突集判断

       else if (GN_point->GN.index != point_pro->GN.index
                && GN_point->path.target_index != point_pro->path.target_index) {//起点终点都不同
                double* wait_time =new double( point_pro->end_time + (pro_length) / pro_path->first.car_v - GN_point->start_time);
                
                delay_func(&(*path).second, k , size,wait_time);//后续时间窗延后
                
                delete wait_time;
            }

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
        if (GN_point.index != 0)//从第二节开始计算转弯时间
        {
            double x1 = GN_point.GN.coordinates.x - previous.GN.coordinates.x;
            double y1 = GN_point.GN.coordinates.y - previous.GN.coordinates.y;

            double x2 = ((*GNs)[GN_point.path.target_index].coordinates.x) - (GN_point.GN.coordinates.x);
            double y2 = ((*GNs)[GN_point.path.target_index].coordinates.y) - (GN_point.GN.coordinates.y);
            // cout << x1 << " " << y1 << endl;
            //计算转弯角度cos值
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

        else if (GN_point.index == -1) {    //-1的话，代表最后一个节点
            GN_point.spend_time = 0;
        }
        
        else {                  //其他的话，代表正常路段的时间

            GN_point.spend_time = GN_point.path.leng / (*path).first.car_v+ (1 - cos) * wheel_time;//路长除以速度+转弯时间
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
    double length_time = 0.0;
    if (pro_path->first.type == 0) { length_time = agv_length / pro_path->first.car_v; }
    else { length_time = fork_length / pro_path->first.car_v; }
    if (k < (size - 2) && n < (pro_size - 2) 
        &&GN_point->path.target_index==point_pro->path.target_index                    //保证此段路不为最后一段路
        && GN_point->end_time != (*path).second[k+1].start_time                 //此节点先到
        && point_pro->end_time < GN_point->end_time
        &&(*pro_path).second[n+1].start_time <= (*path).second[n+1].start_time) {    //但是下一个节点被超了
        bool is_solve = false;     //是否有分叉路
        if (GN_point->GN.index == (*pro_path).second[n+1].path.target_index) {//同一段路

             is_solve=backtrack(path,k,pro_path,n+1,GNs,1.0);
             is_solve = true;
             is_return = true;
        }
        else if (GN_point->GN.index != (*pro_path).second[n + 1].path.target_index) {
            double* wait_time =new double(point_pro->end_time - length_time-GN_point->start_time) ;
            delay_func(&(*path).second, k , size, wait_time);//后续时间窗延后
            delete wait_time;
            is_solve = true;
        }

        if (!is_solve) {              //审核员审核是否解决（没找到分岔路）
            pair<Car_config, pathList>temp = (*paths)[j];
            if (i == ((*paths).size() - 1)) {
                (*paths).push_back(temp);
            }
            else { (*paths).insert((*paths).begin() + i+1 , temp); }
            (*paths)[j].first.index = -1;
            temp.second.clear();
            (*paths)[j].second.clear();
            (*paths)[j].second.shrink_to_fit();
            
        }
           
    }
       
   //else if (k < (size - 2) && n < (pro_size - 2)
   //     &&GN_point->start_time < point_pro->start_time
   //         && (*path).second[k+1].start_time >= (*pro_path).second[n+1].start_time) {
   //         //采取后车减速
   //         if ((*path).first.car_v > pro_path->first.car_v)
   //         {
   //             (*path).first.car_v = pro_path->first.car_v;
   //         }

   //         double moderate_time = (*path).second[k].start_time;
   //         
   //         for (uint m = k; m < (*path).second.size(); m++) {
   //             (*path).second[m].start_time = moderate_time;
   //             (*path).second[m].spend_time = (*path).second[m].path.leng / (*path).first.car_v;
   //             moderate_time += (*path).second[m].spend_time;
   //             (*path).second[m].end_time = moderate_time;
   //         }
   //     } 
    return is_return;
}

//节点冲突
void ASplanner::Generator::node_conflict(car_path* path,uint k ,car_path* pro_path,uint n ,vector<G_Node>* GNs)
{
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    uint size = path->second.size();
    const double* pro_length;
    uint pro_size = pro_path->second.size();
    if (pro_path->first.type == 0) { pro_length = &agv_length; }//AGV车长/速度
    else { pro_length = &fork_length; }//叉车车长/速度

    if (GN_point->GN.index != point_pro->GN.index // 两条路起点不一样
        && GN_point->path.target_index == point_pro->path.target_index //目标点相同
        && GN_point->path.collsion_id != point_pro->path.collsion_id&& (n<pro_size-2)
        &&k>0&&(*pro_path).second[n+1].path.target_index != GN_point->GN.index) {
        auto pro_next = &(*pro_path).second[n + 1];
        if ((GN_point->start_time < point_pro->start_time && GN_point->end_time > point_pro->start_time)
            || (point_pro->start_time < GN_point->start_time && point_pro->end_time > GN_point->start_time )
            ||(point_pro->end_time == GN_point->end_time)  //结束时间相同
            ||(GN_point->start_time < point_pro->start_time && GN_point->end_time > point_pro->end_time)
            ||(GN_point->start_time > point_pro->start_time && GN_point->end_time < point_pro->start_time)) {
            auto pro_next = &(*pro_path).second[n + 1];
            double* wait_time =new double( pro_next->end_time - GN_point->start_time+(*pro_length/pro_path->first.car_v));
            delay_func(&(*path).second, k, size, wait_time);//后续时间窗延后
            
            delete wait_time;
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
            && (GN_point->start_time - point_pro->start_time) < mini_time) {  //但是时间差不够大（小于最小车距时间要求）
            if ((*path).first.car_v != pro_path->first.car_v) {
                (*path).first.car_v = pro_path->first.car_v;
            };     //减速
            
            double cnt_time = (*path).second[k].start_time;
            for (uint m = k; m < size;m++) {
                (*path).second[m].start_time = cnt_time;
                (*path).second[m].spend_time= (*path).second[m].path.leng / path->first.car_v;
                cnt_time += (*path).second[m].spend_time;
                (*path).second[m].end_time = cnt_time;
            }    

            if ((GN_point->spend_time > mini_time)||(GN_point->spend_time == mini_time)) {        //检查一段路时间够不够
                //delay_func(&(*path).second, k , size,&mini_time);//后续时间窗延后
                for (uint m = k; m < size; m++) {
                    (*path).second[m].start_time += mini_time;
                    (*path).second[m].end_time += mini_time;
                }
            }

       else if (point_pro->spend_time < mini_time && k<(size-2)
                &&((*path).second[k+1].spend_time+GN_point->spend_time)>mini_time) { //检查两段路时间够不
                
                double* wait_time=new double(mini_time/*最小时间*/ - GN_point->spend_time + point_pro->end_time-GN_point->start_time);
                delay_func(&(*path).second, k - 1, size, wait_time);//后续时间窗延后    
                /*for (uint m = k-1; m < size; m++) {
                    (*path).second[m].start_time += mini_time;
                    (*path).second[m].end_time += mini_time;     
                }*/
                delete wait_time;
            }

       else if (point_pro->spend_time < mini_time&& k<(size-3)&&n<(pro_size-3)
           && ((*path).second[k + 1].spend_time + GN_point->spend_time) > mini_time) { //三段路时间够

                double *wait_time =new double(mini_time - GN_point->spend_time - (*path).second[k+1].spend_time + point_pro->end_time - GN_point->start_time);
                delay_func(&(*path).second, k - 2, size, wait_time);//后续时间窗延后
                /*for (uint m = k-2; m < size; m++) {
                    (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                }*/
                delete wait_time;
            }
            else {
            
                for (uint m = k; m < size; m++) {
                    (*path).second[m].start_time += mini_time;
                    (*path).second[m].end_time += mini_time;
                }
            }
        }
                                                                       
       else if(( point_pro->start_time > GN_point->start_time)   //优先级低的在前
           && (point_pro->start_time - GN_point->start_time) < mini_time){      //不满足最小安全时间

            bool is_solve = false;
            for (uint ii = 1; ii < (k+1) && ii < (n+1); ii++)//取最小值
            {
                if ((*path).second[k - ii].GN.index != (*pro_path).second[n - ii].GN.index
                    && (*path).second[k - ii].path.target_index == (*pro_path).second[n - ii].path.target_index) {

                    double *wait_time =new double( (*pro_path).second[n - ii].end_time + (mini_time - (*path).second[k - ii].spend_time) - (*path).second[k - ii].start_time);
                    delay_func(&(*path).second, k - ii, (*path).second.size(), wait_time);//后续时间窗延后
                    //for (uint u = (k - ii); u < ((*path).second.size()); u++)//后移时间窗
                    //{
                    //    (*path).second[u].start_time += wait_time;
                    //    (*path).second[u].end_time += wait_time;
                    //}
                    delete wait_time;
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
                (*paths)[j].second.shrink_to_fit();
               std::cout << "优先级高的车在后" << endl;
            }
        }
    }

    else if ((path->first.car_v < pro_path->first.car_v)            //低优先级车速小于优先级高的车
        && GN_point->GN.index == point_pro->GN.index && GN_point->path.target_index == point_pro->path.target_index) {//同一段路

            if (GN_point->start_time - point_pro->start_time > 0//优先级高的车在前
                && GN_point->start_time - point_pro->start_time < mini_time) {//差的不多
                double *wait_time =new double( mini_time - GN_point->start_time + point_pro->start_time);
                
                delay_func(&(*path).second, k, size, wait_time);//后续时间窗延后
                /*for (uint m = k; m < size; m++) {
                (*path).second[m].start_time += wait_time;
                (*path).second[m].end_time += wait_time;
                }*/
                delete wait_time;
            }

            else if (GN_point->start_time < point_pro->start_time ){ //低优先级的车在前
                    //往前回溯
                bool is_solve = false;
                for (uint ii = 1; ii < (k+1) && ii < (n+1); ii++)//取最小值
                {
                    if ((*path).second[k - ii].GN.index != (*pro_path).second[n - ii].GN.index
                        && (*path).second[k - ii].path.target_index == (*pro_path).second[n - ii].path.target_index) {

                        double *wait_time = new double((*pro_path).second[n - ii].end_time +(mini_time - (*path).second[k - ii].spend_time)- (*path).second[k - ii].start_time);
                        
                        delay_func(&(*path).second, k-ii, size, wait_time);//后续时间窗延后
                        //for (uint u = (k - ii); u < ((*path).second.size()); u++)//后移时间窗
                        //{
                        //    (*path).second[u].start_time += wait_time;
                        //    (*path).second[u].end_time += wait_time;
                        //}
                        delete wait_time;
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
                    (*paths)[j].second.shrink_to_fit();
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
    if (n < (pro_size - 2) && k > 0) {//保证n不为最后的点
        auto GN_pointg = &(*path).second[k - 1];
        auto pro_next = &(*pro_path).second[n + 1];
        uint size = path->second.size();
        uint pro_size = pro_path->second.size();
        if (GN_point->path.target_index == point_pro->path.target_index && GN_point->GN.index != point_pro->GN.index
            && point_pro->end_time != pro_next->start_time  //前车在该节点有等待
            && GN_point->end_time > point_pro->end_time && GN_point->end_time < pro_next->start_time){//在前车等待时间路过该节点
            if (pro_next->path.target_index==GN_point->GN.index) {
                replanning_path(path, GN_point, GNs);
                //重新规划
                cout << "同一起点，前车等待时路过该节点" << endl;
                is_return = true;
            }
       
            else if(pro_next->path.target_index != GN_point->GN.index){
                double* wait_time = new double(pro_next->end_time - GN_point->start_time);
                cout << "前车目标点不为该点，等待" << endl;
                delay_func(&(*path).second, k, size, wait_time);//后续时间窗延后
               
                delete wait_time;
            }
        }
   
    }
    return is_return;
}

//库区有车子
bool ASplanner::Generator::store_is_vechel(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs, vector<pair<Car_config, pathList>>* paths, uint i, uint j) {
    bool is_return = false;
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    uint size = path->second.size();
    uint pro_size = pro_path->second.size();
    const double* pro_length;
    if (pro_path->first.type == 0) { pro_length = &agv_length; }
    else { pro_length = &fork_length; }
    
    if (k>2&&(k < size - 3)&& (*path).second[k + 1].GN.name.last_id == 0 &&( n < pro_size - 3) &&(*path).second[k+2].GN.name.last_id !=0) {  //该车需要进入断头路
        path_point* GN_out= nullptr;//初始化
        for (uint count = k+2; count<(*path).second.size();count++) {
            if ((*path).second[count].GN.name.last_id == 0) {
                GN_out = &(*path).second[count];
                 break;
            }
        }
        path_point* pro_out=nullptr;
         
        auto pro_next = &(*pro_path).second[n+1];
        if (n < (pro_size - 3) && (*pro_path).second[n + 1].GN.name.last_id == 0 &&(*pro_path).second[n + 2].GN.name.last_id != 0 //优先级高的车要进入断头路
        &&pro_next->GN.name.sec_id == (*path).second[k+2].GN.name.sec_id) {
            for (uint count = n+2; count < (*pro_path).second.size(); count++) {
                if ((*pro_path).second[count].GN.name.last_id == 0) {
                    pro_out=&(*pro_path).second[count];
                    break;
                }
            }
            if (pro_out == nullptr) { cout << "the same middle point,cant come in!!!" << endl; exit(1); }
            if (GN_point->end_time < point_pro->start_time && GN_out->start_time > point_pro->start_time) {//优先级低的车先进入断头路
                   //改变优先级
                    pair<Car_config, pathList>temp = (*paths)[j];
                    if (i == ((*paths).size() - 1)) {
                        (*paths).push_back(temp);
                    }
                    else { (*paths).insert((*paths).begin() + i + 1, temp); }
                    (*paths)[j].first.index = -1;
                    temp.second.clear();
                    (*paths)[j].second.clear();
                    (*paths)[j].second.shrink_to_fit();
                    cout << "仓库区有车子，调整优先级！" << endl;

            }

            else if (GN_point->end_time > point_pro->end_time && GN_point->end_time < pro_out->start_time) {          //优先级高的车先进入断头路
                    //判断出站方向,
                if (pro_out->path.target_index == GN_point->GN.index) {   //冲突
                    replanning_path(path, GN_point, GNs);
                    is_return = true;
                    cout << "断头路重新规划" << endl;
                }

                else if (pro_out->path.target_index != GN_point->GN.index) {//最简单的情况
                    double* wait_time = new double(pro_out->end_time + (*pro_length) / (pro_path->first.car_v) - (*path).second[k].start_time);
                    
                    delay_func(&(*path).second, k, size, wait_time);//后续时间窗延后
                    /*for (uint m = k; m < size; m++) {
                        (*path).second[m].start_time += wait_time;
                        (*path).second[m].end_time += wait_time;
                    }*/
                    delete wait_time;
                    cout << "断头路执行等待" << endl;
                }
            }
        }
    }
return is_return;
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
   (*path).second.erase((*path).second.begin() + (*start_point_index), (*path).second.begin() + (*target_point_index)+1);

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
         (*path).first.middle_point[c+1].first += del_size;
         (*path).first.middle_point[c].first += del_size;
     }
    
}

void ASplanner::Generator::dont_in(car_path* path, path_point* GN_point, vector<G_Node>* GNs) {






}
//任务调度
string ASplanner::Generator::task_scheduling(vector<pair<Car_config, pathList>> GNLs,string Filename, vector<G_Node>* GNs) {
    const char* p = &Filename[0];
    XMLDocument doc_agv;
    tinyxml2::XMLError if_success = doc_agv.LoadFile(p);
    if (if_success != 0) {
        std::cout << "open file failed" << endl;
        ; exit(1);
    }
    doc_agv.Error();
    uint count = 0;
    XMLElement* agv_titleElement = doc_agv.FirstChildElement();
    XMLElement* agv = agv_titleElement->FirstChildElement("agv");
    vector<pair<Car_config, pathList>> task_vechal;
    while (agv) {
        car_path temp_car;
        const XMLAttribute* start_point = agv->FindAttribute("start_point");
        const XMLAttribute* end_point = agv->FindAttribute("end_point");
        const XMLAttribute* type = agv->FindAttribute("type");
        const XMLAttribute* car_v = agv->FindAttribute("car_v");
        const XMLAttribute* car_name = agv->FindAttribute("name");
        Car_config temp = { std::atoi(type->Value()) ,GNLs.size()+ count,car_name->Value(),{},std::stod(car_v->Value())};
        temp_car.first = temp;
        uint start = findpoint(start_point->Value(), GNs);
        uint end = findpoint(end_point->Value(), GNs);
        temp_car.second = findPath((*GNs)[start], (*GNs)[end], GNs);
        init_time_windows(0.0, &temp_car, GNs);
        task_vechal.push_back(temp_car);
        temp_car.second.clear();
        count++;
        agv = agv->NextSiblingElement("agv");
    }
    
    uint select = 0;
    cout << "select Shortest path please enter '1'" << endl;
    cout << "select Shortest time please enter '0'" << endl;
    cout << "select=";
    /*cin >> select;*/
    select = 0;
    if (select == 1) {
        std::sort(task_vechal.begin(), task_vechal.end(), [](const pair<Car_config, pathList>&a, const pair<Car_config, pathList>&b) {
            double a_add = 0.0;
            for (uint i = 0; i < a.second.size(); i++) {
                a_add += a.second[i].spend_time;
            }
            double b_add = 0.0;
            for (uint i = 0; i < b.second.size(); i++) {
                b_add += b.second[i].spend_time;
            }
            return a_add < b_add;
            // 根据value成员变量升序排序
        });
        return GNLs[0].first.car_name;
    }

    else {
        for (uint i = 0; i < task_vechal.size(); i++) {
            GNLs.push_back(task_vechal[i]);
            conflict_check(&GNLs, GNs);
            task_vechal[i] = (GNLs)[GNLs.size()-1];
            GNLs.pop_back();
        }
        std::sort(task_vechal.begin(), task_vechal.end(), [](const pair<Car_config, pathList>& a, const pair<Car_config, pathList>& b) {
            double a_add = 0.0;
            for (uint i = 0; i < a.second.size(); i++) {
                a_add += a.second[i].path.leng;
            }
            double b_add = 0.0;
            for (uint i = 0; i < b.second.size(); i++) {
                b_add += b.second[i].path.leng;
            }
            return a_add < b_add;
            });
        return GNLs[0].first.car_name;
    }
    GNLs.clear();
    GNLs.shrink_to_fit();
}

bool ASplanner::Generator::backtrack(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs,double pro_length) {
    bool is_solve=false;
    auto GN_point = &(*path).second[k];
    uint pro_size =pro_path->second.size() ;
    uint size = path->second.size();
    for (uint m = k; m > 0; m--) {
        if (is_solve) { break; }
        auto GN_pointg = (*path).second[m];
        if ((n + k - m < pro_size - 2) && GN_pointg.GN.index != pro_path->second[n + k - m].path.target_index)//优先级高的车此段路与本车无冲突冲突
        {
            double* wait_time = new double(pro_path->second[n + k - m].end_time + (pro_length / pro_path->first.car_v) - (path->second[m].start_time));
            delay_func(&(*path).second, m, size, wait_time);
            delete wait_time;
            is_solve = true;
        }
        else if ((n + k - m < pro_size - 2) && GN_pointg.GN.index == pro_path->second[n + k - m].path.target_index) {  //优先级高的车此段路还有本车有冲突
            for (auto it = GN_pointg.GN.link_edges.begin(); it != GN_pointg.GN.link_edges.end(); it++) {//遍历k-1段路的连接关系
                if (is_solve) { break; }
                if (m > 0 && (*it).target_index != (*path).second[m - 1].GN.index   //不为上一段路
                    && (*it).target_index != (*path).second[m].path.target_index) { //不为下一段路（出度大于2）
                    //遍历GN_pointg的target这个点的连接关系
                    for (auto tt = (*GNs)[(*it).target_index].link_edges.begin(); tt != (*GNs)[(*it).target_index].link_edges.end(); tt++) {
                        uint c = 0;
                        for (c; c < (path->first.middle_point.size() - 1); c++) {
                            if ((path->first.middle_point[c].first < GN_point->index || path->first.middle_point[c].first == GN_point->index)
                                && GN_point->index < path->first.middle_point[c + 1].first) {
                                break;
                            }
                        }
                        if ((*tt).target_index == (*path).second[m + 1].path.target_index) {

                            path_point path_point1 = { 0,GN_pointg.GN,*it,GN_pointg.start_time,0,0 };
                            path_point1.spend_time = path_point1.path.leng / path->first.car_v;
                            path_point1.end_time = path_point1.start_time + path_point1.spend_time;

                            if (path_point1.end_time < pro_path->second[n + k - m].start_time) {
                                (*path).second.erase((*path).second.begin() + m, (*path).second.begin() + m + 1);
                                (*path).second.insert((*path).second.begin() + m, path_point1);
                                path_point path_point2 = { 0,(*GNs)[(*tt).source_index],*tt, pro_path->second[n + k - m].end_time,0,0 };
                                path_point2.spend_time = path_point2.path.leng / path->first.car_v;
                                path_point2.end_time = path_point2.spend_time + path_point2.start_time;
                                (*path).second.insert((*path).second.begin() + m + 1, path_point2);

                                uint ii = 0;
                                for (auto p = path->second.begin(); p != path->second.end(); p++) {
                                    (*p).index = ii;
                                    ii++;
                                }
                                double wait_time = path_point2.end_time - (*path).second[m + 2].start_time;
                                delay_func(&(*path).second, m + 2, path->second.size(), &wait_time);
                                is_solve = true;
                                break;
                            }
                        }

                        else if ((*tt).target_index == GN_pointg.path.target_index) {
                            path_point path_point1 = { 0,GN_pointg.GN,*it,GN_pointg.start_time,0,0 };
                            path_point1.spend_time = path_point1.path.leng / path->first.car_v;
                            path_point1.end_time = path_point1.start_time + path_point1.spend_time;

                            if (path_point1.end_time < pro_path->second[n + k - m].start_time) {
                                (*path).second.erase((*path).second.begin() + m);
                                (*path).second.insert((*path).second.begin() + m, path_point1);

                                path_point path_point2 = { 0,(*GNs)[(*tt).source_index],*tt, pro_path->second[n + k - m].end_time,0,0 };
                                path_point2.spend_time = path_point2.path.leng / path->first.car_v;
                                path_point2.end_time = path_point2.spend_time + path_point2.start_time;
                                (*path).second.insert((*path).second.begin() + m + 1, path_point2);

                                uint ii = 0;
                                for (auto p = path->second.begin(); p != path->second.end(); p++) {
                                    (*p).index = ii;
                                    ii++;
                                }
                                double wait_time = path_point2.end_time - (*path).second[m + 2].start_time;
                                delay_func(&(*path).second, m + 2, path->second.size(), &wait_time);
                                for (uint v = c + 1; v < path->first.middle_point.size(); v++) {
                                    (*path).first.middle_point[v].first += 1;
                                }
                                is_solve = true;
                                break;
                            }
                            
                        }//折返型避障
                        else if ((*tt).target_index == GN_pointg.GN.index) {
                            path_point path_point1 = { 0,GN_pointg.GN,*it,GN_pointg.start_time,0,0 };
                            path_point1.spend_time = path_point1.path.leng / path->first.car_v;
                            path_point1.end_time = path_point1.start_time + path_point1.spend_time;

                            /*if (path_point1.end_time < pro_path->second[n + k - m].start_time) {*/

                            (*path).second.insert((*path).second.begin() + m, path_point1);
                            path_point path_point2 = { 0,(*GNs)[(*tt).source_index],*tt, pro_path->second[n + k - m].end_time,0,0 };
                            path_point2.spend_time = path_point2.path.leng / path->first.car_v;
                            path_point2.end_time = path_point2.spend_time + path_point2.start_time;
                            (*path).second.insert((*path).second.begin() + m + 1, path_point2);

                            uint ii = 0;
                            for (auto p = path->second.begin(); p != path->second.end(); p++) {
                                (*p).index = ii;
                                ii++;
                            }
                            double wait_time =  path_point2.end_time- path_point1.start_time ;
                            delay_func(&(*path).second, m + 2, path->second.size(), &wait_time);
                            for (uint v = c + 1; v < path->first.middle_point.size(); v++) {
                                (*path).first.middle_point[v].first += 2;
                            }

                            is_solve = true;
                            break;
                            //}
                        }
                    }
                }
            }
        }//不为岔路
    }
    return is_solve;
}


int  ASplanner::Generator::findpoint(const char* point, vector<G_Node>* GNs)
{
    char str[20] = { 0 };
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
        token = strtok_s(NULL, "-", &next_token);
    }
    //cout<<"起始/目标点" << s[0] << "-" << s[1] << "-" << s[2] << endl;
    vector<G_Node>::iterator it;
    int i = 0;
    for (it = GNs->begin(); it != GNs->end(); it++)
    {
        if (it->name.main_id == atoi(s[0].c_str()) && it->name.sec_id == atoi(s[1].c_str()) && it->name.last_id == atoi(s[2].c_str()))
        {
            i = it->index;
            break;
        }
    }
    if (it == GNs->end()) {
        cout << "cant find the point:" << point << "system out" << endl;
        exit(1);
    }
    return i;
}

