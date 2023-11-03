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
    for (uint i = 0; i < (*paths).size(); i++) {    // iС������
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
            for (uint k = 0; k < (*paths)[i].second.size() && !flag; k++)//�ȶԣ��������㡣k:��i��С����·���ڵ�λ��
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
                    for (uint n = 0; (*paths)[j].first.index != -1&& n < (*paths)[j].second.size(); n++)//���ȼ����ߵĹ켣��
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

                       // �ڵ��ͻ
                        /*if ((*paths)[j].second.size() >2 ) {
                             node_conflict(&(*paths)[i],k,&(*paths)[j],n, GNs);
                        }*/

                        //ͬ���ͻ
                        if ((*paths)[j].first.index !=-1) {
                            bool is_collidiong_return = colliding_conflict(&(*paths)[i], k, &(*paths)[j], n, GNs, paths, i, j);
                        
                            if (is_collidiong_return)
                            {
                                flag = true;
                                break;
                            }
                        }
                        //��ͻ���жϼ������
                        if ((*paths)[j].first.index != -1) {
                            bool is_collsion_return = collsion_collection(&(*paths)[i], k, &(*paths)[j], n, GNs,paths, i, j);
                            if (is_collsion_return)
                            {
                                flag = true;
                                break;
                            }
                        }
                        ////�����ͻ
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
                }//��m
            }//��i��k
        }//whileѭ��
        std::cout<<(*paths)[i].first.index <<" ��" << "whileѭ������ ��" << cycle_count << endl;
        if (cycle_count == 10) { std::cout << "ѭ���������࣬�Զ��˳�����" << endl; exit(1); }
        for (auto t = GNs->begin(); t != GNs->end(); t++)//�ָ���ͼ��·��Ϣ
        {
            for (auto edge : t->link_edges)
            {
                edge.state = true;
            }
        }
    }
    cout << "������" << ff << endl;
    return 0;
}
ASplanner::pathList  ASplanner::Generator::findPath(G_Node source_, G_Node target_, vector<G_Node>* GNs)
{
    Node* current = nullptr;
    NodeSet openSet, closedSet; //openSet��ѡ��Ҷ�ڵ�,closedSet�Ѿ���ѡ��ķ�Ҷ�ڵ�
    openSet.reserve(100);       //����100��Ԫ�ص��ڴ�ռ�
    closedSet.reserve(100);     //����100��Ԫ�ص��ڴ�ռ�
    openSet.push_back(new Node(source_));

    while (!openSet.empty()) 
    {//����̽������
        auto current_it = openSet.begin();
        current = *current_it;
        for (auto it = openSet.begin(); it != openSet.end(); it++)
        {
            auto node = *it;
            //���ҵ�ǰ�ڵ㸽�����·���Ľڵ㲢������ӵ�Ҷ�ڵ�
            if (node->getScore() <= current->getScore()) 
            {
                current = node;
               // cout << "�ڵ� "<<*it << " " << current->GN.name.main_id << "-" << current->GN.name.sec_id << "-" << current->GN.name.last_id << endl;
                current_it = it;
            }
        }
        //��ǰ���������Ŀ��ڵ����꣬���ҵ�Ŀ��ڵ�
        if (current->coordinates == target_.coordinates) {
            break;
        }

        closedSet.push_back(current);   //���ҵ������·��������ڵ�ѹ������������ӵ�Ҷ�ڵ�
        openSet.erase(current_it);      //����Ѿ���ѡ��Ľڵ㡪��closedSet�Ѿ���ѡ��ķ�Ҷ�ڵ�
        
        for (auto edge : current->GN.link_edges)//auto��ָ������ǰ�ڵ����Χ���ڽڵ�
        {
            if (edge.state) 
            {
                if (findNodeOnList(closedSet, edge.target_index)) {//���������������˵�
                    continue;
                }
                //·�γ��ȴ�����ۣ�����������޸Ĵ��ۺ�����Ҳ�ɼ�����������Ϣ�������Լ�����
                double totalCost = current->G + edge.leng;
                Node* successor = findNodeOnList(openSet, edge.target_index);//�Ƿ��ں�ѡ·�������ڣ���Ӵ��µ㣻
                if (successor == nullptr)
                {//̽���µ�
                    successor = new Node((*GNs)[edge.target_index], current);
                    successor->G = totalCost;
                    successor->H = heuristic(successor->coordinates, target_.coordinates)/100;
                    openSet.push_back(successor);
                }
                else if (totalCost < successor->G) {//��������ҵ��ĵ���ۻ�С
                    successor->parent = current;
                    successor->G = totalCost;
                }
            }
        }
    }
    G_NodeList path;
    pathList time_path;
    while (current != nullptr) {//������ѯ
        path.push_back(current->GN);
        current = current->parent;
    }
    reverse(path.begin(), path.end());//����
    for (uint i = 0; i < path.size(); i++) {
        path_point temp_t;      //�켣������
        temp_t.GN = path[i];
        if (i + 1 == path.size()) {
            temp_t.index = -1;//�������һ����
        }
        else {
            temp_t.index = i;//�켣������
            for (auto edge : temp_t.GN.link_edges) {
                if (edge.target_index == path[i + 1].index)//·��ƥ��
                {
                    temp_t.path = edge;//�ҵ�Ŀ��·��
                }
            }
        }
        time_path.push_back(temp_t);    //��·���ĵĸ���վ��ѹ��time_pathջ��
    }

    releaseNodes(openSet);  //�ͷ�ջ�ռ�
    releaseNodes(closedSet);//�ͷ�ջ�ռ�
    /*if (time_path[time_path.size()-1].GN.index != target_.index)
    {
        cout << "Ѱ·ʧ�ܣ�" << endl;
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

//�ͷ�ջ�����пռ�
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
//�����ͻ
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
                && path->second[k - 1].GN.index != pro_path->second[n + 1].path.target_index) { //������һ����̽�ɷ���
                double wait_time = pro_path->second[n + 1].end_time + (pro_length / pro_path->first.car_v) - (path->second[k - 1].start_time);
                for (uint m = k - 1; m < size; m++) {
                    (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                }
                //�����ͻ������һ�ڵȴ�
                cout << "�����ͻ,����һ��" << endl;
                is_conflict = true;
                is_solve = true;
            }
       else if (k > 1 && pro_size > 3 && point_pro->index < (pro_size - 3)
                && path->second[k - 2].GN.index != pro_path->second[n + 2].path.target_index) {//��������·�����ܲ���ͨ���ȴ����
                
                double wait_time = pro_path->second[n + 2].end_time + (pro_length / pro_path->first.car_v) - (path->second[k - 2].start_time);
                for (uint m = k - 2; m < size; m++) {
                    (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                } cout << "�����ͻ,��������" << endl;
                is_conflict = true;
                is_solve = true;
            }
       else if (k > 2 && pro_size > 4 && point_pro->index < (pro_size - 4)
                && path->second[k - 3].GN.index != pro_path->second[n + 3].path.target_index) {
                
                double wait_time = pro_path->second[n + 3].end_time + (pro_length / pro_path->first.car_v) - (path->second[k - 3].start_time);
                for (uint m = k - 3; m < size; m++) {
                    (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                } cout << "�����ͻ,��������" << endl;
                is_conflict = true;
                is_solve = true;
            }
            else {      //��Ҫ���¹滮  ��·
                uint end_index = ((*path).second[size - 1].GN.index);  //���һ��Ԫ�������ŵ�ַ
                uint start_index = ((*path).second[0].GN.index);

                if (((path->first.car_v > pro_path->first.car_v) || (path->first.car_v == pro_path->first.car_v)) && k > 0){
                    
                    auto GN_pointg = &(*path).second[k - 1];
                    replanning_path(path,GN_pointg,GNs);
                    
                    std::cout << "�����ͻ�滮���" << endl;
                    is_conflict = true;
                    is_solve = true;
                }

                else if ((path->first.car_v < pro_path->first.car_v)) {

                    auto GN_pointg = &(*path).second[0];
                    if (k > 1) { GN_pointg = &(*path).second[k - 2]; }

                    replanning_path(path, GN_pointg, GNs);

                    std::cout << "�����ͻ�滮���" << endl;
                    is_conflict = true;
                    is_solve = true;
                }
            }
            if (!is_conflict) {
                cout << "δ��������ͻ" << endl;
                exit(1);
            }
        }
    }
    return is_conflict;
}
//��ͻ��
bool ASplanner::Generator::collsion_collection(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs,vector<pair<Car_config, pathList>>* paths, uint i, uint j)
{
     bool is_return=false;
     auto point_pro = &(*pro_path).second[n];
     auto GN_point = &(*path).second[k];
     uint pro_size = pro_path->second.size();
     uint size = path->second.size();
     double pro_length=0.0;
     if (path->first.type == 0) { pro_length = agv_length; }//AGV����
     else { pro_length= fork_length ; }//�泵����

    if (GN_point->path.collsion_id == point_pro->path.collsion_id &&GN_point->path.collsion_id != 0    //��ͻ������ͬ
        && !((GN_point->GN.index == point_pro->GN.index && GN_point->path.target_index == point_pro->path.target_index)
        || (GN_point->GN.index == point_pro->path.target_index && GN_point->path.target_index == point_pro->GN.index))){//������ͬһ·��
    
        if ((GN_point->start_time < ((point_pro->end_time)/*+ length_time*/)
            && GN_point->start_time > point_pro->start_time)
            || (GN_point->end_time < ((point_pro->end_time) /*+ length_time*/)
                && GN_point->end_time > point_pro->start_time)
            ||((GN_point->start_time < ((point_pro->start_time)/*+ length_time*/)
                && GN_point->end_time > point_pro->end_time)
            ||((GN_point->start_time > ((point_pro->start_time)/*+ length_time*/)
                && GN_point->end_time <  point_pro->end_time)) )){ //����ʱ�佻���ص� 
            
            if (GN_point->path.target_index == point_pro->GN.index) {                 //���0�����յ�Ϊǰ�����
                double wait_time = point_pro->end_time + ((pro_length) / pro_path->first.car_v) - (GN_point->start_time);
                for (uint m = k; m < size; m++) {
                    //����ʱ�䴰�Ӻ�
                   (*path).second[m].start_time += wait_time;
                    (*path).second[m].end_time += wait_time;
                }
                cout << "���յ�Ϊǰ����㣬ʱ�䴰����" << endl;
            }
            else if (GN_point->GN.index == point_pro->path.target_index) {          //���1��ǰ��Ŀ����Ǻ����
                if (k > 0 && pro_size > 2 &&(point_pro->index) < (pro_size - 2)//�����㹻
                    && path->second[k - 1].GN.index != pro_path->second[n + 1].path.target_index) {//����һ������̽��
                    double wait_time=pro_path->second[n + 1].end_time + ((pro_length) / pro_path->first.car_v) - path->second[k - 1].start_time;
                    for (uint m = k - 1; m < size; m++) {                 //����ʱ�䴰�Ӻ�
                        (*path).second[m].start_time += wait_time;
                        (*path).second[m].end_time += wait_time;
                        }
                    is_return = true;
                }//��̽����

            else if (k > 1 && pro_size > 3 && (point_pro->index) < (pro_size - 3)
                    && path->second[k - 2].GN.index != pro_path->second[n + 2].path.target_index ) {//������������̽
                    double wait_time = pro_path->second[n + 2].end_time + (pro_length) / pro_path->first.car_v - path->second[k - 2].start_time;
                        for (uint m = k - 2; m < size; m++) {
                            (*path).second[m].start_time += wait_time;
                            (*path).second[m].end_time += wait_time;
                        }
                        is_return = true;
                 }//��̽����

                else {   //��̽���ˣ�û�취�ˡ����¹滮
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

            else if(GN_point->GN.index == point_pro->GN.index){    //���2�������ͬ���յ㲻ͬ����Ϊ�������
                bool is_solve = false;
                if (GN_point->start_time < point_pro->start_time) {//���ȼ��͵ĳ��ȵ�
                    for (uint ii = 1; ii < (k +1)&& ii < (n+1); ii++)//ȡ��Сֵ
                    {
                        if ((*path).second[k - ii ].GN.index != (*pro_path).second[n - ii ].GN.index
                            && (*path).second[k - ii ].path.target_index == (*pro_path).second[n - ii ].path.target_index)//�ҷֲ�·��
                        {
                            double length_time = 0.0;
                            if (pro_path->first.type == 0) { length_time = agv_length / pro_path->first.car_v; }
                            else { length_time = fork_length / pro_path->first.car_v; }

                            double wait_time = fabs(((*path).second[k - ii].start_time) - ((*pro_path).second[n - ii].end_time)) + length_time;
                            for (uint u = (k - ii ); u < ((*path).second.size()); u++)//����ʱ�䴰
                            {
                                (*path).second[u].start_time += wait_time;
                                (*path).second[u].end_time += wait_time;
                            }
                            is_return = true;
                            is_solve=true;
                            break;
                        }
                    }
                //  ���û���ҵ��ֲ�·��:���������.....
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
                else {//���ȼ��ߵĳ��ȵ�
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
                &&GN_point->path.target_index==point_pro->path.target_index){//���3����㲻ͬ���յ���ͬ��ִ�еȴ�
                
                double set_time = point_pro->end_time - GN_point->start_time +((pro_length) + mini_distance) / pro_path->first.car_v;//�������ʱ��

                cout<< "��ͻ��ִ�еȴ�" << endl;
                for (uint m = k; m < size; m++)//�����ĵ�ȫ�����
                {  (*path).second[m].start_time += set_time; //����ʱ�䴰 
                   (*path).second[m].end_time += set_time;
                }
            }
            //��ͻ���ж�
        }
    }
    return is_return;
}
//ʱ�䴰��ʼ��
void ASplanner::Generator::init_time_windows(double time_cnt,pair<Car_config,pathList> *path,vector<G_Node>* GNs)
{
    path_point previous;//��һ����
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

        if (GN_point.index == 0) {      //0�Ļ����ǵ�һ���ڵ㣬��������ʱ��
            
            if ((*path).first.type==0){
                double acc_leng =( agv_acc * pow(((*path).first.car_v / agv_acc), 2))/2;
                
                    //���ٶ�ʱ��  +  ����ʱ��
                GN_point.spend_time = (*path).first.car_v / agv_acc + (GN_point.path.leng - acc_leng) / (*path).first.car_v;
                
            }

            else {
                double acc_leng = (fork_acc * pow(((*path).first.car_v / fork_acc), 2)) / 2;// �����ٶȳ���
                 //���ٶ�ʱ�� +  ����ʱ��
                GN_point.spend_time = (*path).first.car_v / fork_acc + (GN_point.path.leng - acc_leng) / (*path).first.car_v;
            }
        }

        else if (GN_point.index == -1) {    //-1�Ļ�������û�нڵ�
            GN_point.spend_time = 0;
        }
        
        else {                  //�����Ļ�����������·�ε�ʱ��

            GN_point.spend_time = GN_point.path.leng / (*path).first.car_v+ (1 - cos) * wheel_time;//����ת�� 
        }
        GN_point.start_time = time_cnt ;//��ʼʱ��
        time_cnt += GN_point.spend_time ;
        GN_point.end_time = time_cnt;
        previous = GN_point;
    }
}

//ͬ���ͻ,��Ҫ������ȼ��͵ĳ���ȴ���������
bool ASplanner::Generator::colliding_conflict(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs, vector<pair<Car_config, pathList>>* paths,uint i,uint j)
{   
    bool is_return = false;
    car_path new_path = {};
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    uint pro_size = pro_path->second.size();
    uint size = path->second.size();
    if(GN_point->GN.index == point_pro->GN.index 
        &&GN_point->path.target_index == point_pro->path.target_index){     //ͬһ��·��
        
        if (k < (size - 2) && n < (pro_size - 2)                           //��֤�˶�·��Ϊ���һ��·
            && GN_point->start_time < point_pro->start_time                    //�˽ڵ��ȵ�
            && (*path).second[k + 1].start_time >(*pro_path).second[n + 1].start_time) {    //������һ���ڵ㱻����
            bool is_solve = false;     //�Ƿ��зֲ�·
            for (uint ii = 1; ii < (k+1) && ii < n; ii++)//ȡ��Сֵ
            {
                if ((*path).second[k - ii].GN.index != (*pro_path).second[n - ii].GN.index
                    && (*path).second[k - ii].path.target_index == (*pro_path).second[n - ii].path.target_index) {

                    double length_time = 0.0;
                    if (pro_path->first.type == 0) { length_time = agv_length / pro_path->first.car_v; }
                    else { length_time = fork_length / pro_path->first.car_v; }
                    double wait_time = ((*pro_path).second[n - ii].end_time) - ((*path).second[k - ii].start_time) + length_time;
                    for (uint u = (k - ii); u < ((*path).second.size()); u++)//����ʱ�䴰
                    {
                        (*path).second[u].start_time += wait_time;
                        (*path).second[u].end_time += wait_time;
                    }
                    is_solve = true;
                    is_return = true;
                    break;
                }
            }

            if (!is_solve) {              //���Ա����Ƿ���
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
            //��ȡ�󳵼���
            cout << "����ִ�еȴ���" << (*path).first.car_v << "m/s����Ϊ" << pro_path->first.car_v << "m/s" << endl;
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

//�ڵ��ͻ
void ASplanner::Generator::node_conflict(car_path* path,uint k ,car_path* pro_path,uint n ,vector<G_Node>* GNs)
{
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    const double* pro_length;
    if (pro_path->first.type == 0) { pro_length = &agv_length; }//AGV����/�ٶ�
    else { pro_length = &fork_length; }//�泵����/�ٶ�

    if (GN_point->GN.index != point_pro->path.source_index // ����·��㲻һ��
        && (*path).second[k].path.target_index == point_pro->path.target_index //Ŀ�����ͬ
        && GN_point->path.collsion_id != point_pro->path.collsion_id ){
        double delt_time = fabs(GN_point->end_time - point_pro->end_time);
        if (delt_time < ((mini_distance + *pro_length) / pro_path->first.car_v)) {   //ʱ������
            double wait_time = point_pro->end_time + (mini_distance + (*pro_length)) / pro_path->first.car_v - (GN_point->spend_time) - (GN_point->start_time);
            for (uint m = k; m < (*path).second.size(); m++) {
                
                (*path).second[m].start_time += wait_time;
                (*path).second[m].end_time += wait_time;
            }
        }
    }
}
//��С����
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

    if (((path->first.car_v > pro_path->first.car_v) || (path->first.car_v == pro_path->first.car_v))   //���ٴ��ڻ�������ȼ��ߵĳ�
        &&GN_point->GN.index==point_pro->GN.index && GN_point->path.target_index== point_pro->path.target_index//ͬһ��·
        &&GN_point->index <(size-2) && point_pro->index<(pro_size-2)) {     //��Ϊ���һ��·
        
        if ((GN_point->start_time > point_pro->start_time)         //���ȼ��ߵĳ���ǰ
            && (GN_point->start_time - point_pro->start_time) < mini_time) {  //����ʱ������
            (*path).first.car_v = pro_path->first.car_v;      //����
            
            double cnt_time = (*path).second[k].start_time;
            for (uint m = k; m < size;m++) {

                (*path).second[m].start_time = cnt_time;
                (*path).second[m].spend_time= (*path).second[m].path.leng / path->first.car_v;
                cnt_time += (*path).second[m].spend_time;
                (*path).second[m].end_time = cnt_time;
            }    

            if ((GN_point->spend_time > mini_time)||(GN_point->spend_time == mini_time)) {        //һ��·ʱ�乻����
                for (uint m = k; m < size; m++) {
                    (*path).second[m].start_time += mini_time;
                    (*path).second[m].end_time += mini_time;
                }
            }

       else if (point_pro->spend_time < mini_time && k<(size-2)
                &&((*path).second[k+1].spend_time+GN_point->spend_time)>mini_time) { //����·ʱ�乻��
                
                double wait_time=mini_time/*��Сʱ��*/ - GN_point->spend_time + point_pro->end_time-GN_point->start_time;
                    
                for (uint m = k-1; m < size; m++) {
                    (*path).second[m].start_time += mini_time;
                    (*path).second[m].end_time += mini_time;     
                }
            }

       else if (point_pro->spend_time < mini_time&& k<(size-3)&&n<(pro_size-3)
           && ((*path).second[k + 1].spend_time + GN_point->spend_time) > mini_time) { //����·ʱ�乻��

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
                                                                       
       else if(( point_pro->start_time > GN_point->start_time)   //���ȼ��͵���ǰ
           && (point_pro->start_time - GN_point->start_time) < mini_time){      //���ȼ��ߵĳ��ں�

            bool is_solve = false;
            for (uint ii = 1; ii < (k+1) && ii < (n+1); ii++)//ȡ��Сֵ
            {
                if ((*path).second[k - ii].GN.index != (*pro_path).second[n - ii].GN.index
                    && (*path).second[k - ii].path.target_index == (*pro_path).second[n - ii].path.target_index) {

                    double wait_time = (*pro_path).second[n - ii].end_time + (mini_time - (*path).second[k - ii].spend_time) - (*path).second[k - ii].start_time;
                    for (uint u = (k - ii); u < ((*path).second.size()); u++)//����ʱ�䴰
                    {
                        (*path).second[u].start_time += wait_time;
                        (*path).second[u].end_time += wait_time;
                    }
                    is_solve = true;
                    is_return = true;
                    break;
                }
            }

            if (!is_solve) { //�������ȼ�
                pair<Car_config, pathList>temp = (*paths)[j];
                if (i == ((*paths).size() - 1)) {
                    (*paths).push_back(temp);
                }
                else { (*paths).insert((*paths).begin() + i +1, temp); }
                (*paths)[j].first.index = -1;
                temp.second.clear();
                (*paths)[j].second.clear();
               std::cout << "���ȼ��ߵĳ��ں�" << endl;
            }
        }
    }

    else if ((path->first.car_v < pro_path->first.car_v)            //����С�����ȼ��ߵĳ�
        && GN_point->GN.index == point_pro->GN.index && GN_point->path.target_index == point_pro->path.target_index) {//ͬһ��·

            if (GN_point->start_time - point_pro->start_time > 0//���ȼ��ߵĳ���ǰ
                && GN_point->start_time - point_pro->start_time < mini_time) {//��Ĳ���
                double wait_time = mini_time - GN_point->start_time + point_pro->start_time;
                for (uint m = k; m < size; m++) {
                (*path).second[m].start_time += wait_time;
                (*path).second[m].end_time += wait_time;
                }
            }

            else if (GN_point->start_time < point_pro->start_time ){ //�����ȼ��ߵĳ��Ƚ�����ʱ�ջ���
                    //ʱ�ջ���
                bool is_solve = false;
                for (uint ii = 1; ii < (k+1) && ii < (n+1); ii++)//ȡ��Сֵ
                {
                    if ((*path).second[k - ii].GN.index != (*pro_path).second[n - ii].GN.index
                        && (*path).second[k - ii].path.target_index == (*pro_path).second[n - ii].path.target_index) {

                        double wait_time = (*pro_path).second[n - ii].end_time +(mini_time - (*path).second[k - ii].spend_time)- (*path).second[k - ii].start_time;
                        for (uint u = (k - ii); u < ((*path).second.size()); u++)//����ʱ�䴰
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
                    cout << "�����ȼ��ĳ���ǰ���������ȼ���" << endl;
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
            && point_pro->end_time != pro_next->start_time  //ǰ���ڸýڵ��еȴ�
            && GN_point->end_time > point_pro->end_time && GN_point->end_time < pro_next->start_time) { //��ǰ���ȴ�ʱ��·���˽ڵ�
            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//ɾ����ײ·�ε�Ŀ���
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
           //���¹滮
            is_return = true;
        }
    }
    return is_return;
}

//�����г���
void ASplanner::Generator::store_is_vechel(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs, vector<pair<Car_config, pathList>>* paths, uint i, uint j) {
    bool is_return = false;
    auto GN_point = &(*path).second[k];
    auto point_pro = &(*pro_path).second[n];
    uint size = path->second.size();
    uint pro_size = pro_path->second.size();
    const double* pro_length;
    if (pro_path->first.type == 0) { pro_length = &agv_length; }
    else { pro_length = &fork_length; }
    if (k < (size - 1)&&(*GNs)[(*path).second[k + 1].path.target_index].name.last_id !=0) {  //�ó���Ҫ�����ͷ·
        path_point* GN_out= nullptr;//��ʼ��
        for (uint count = k; count<(*path).second.size();count++) {
            if ((*path).second[count].GN.name.last_id == 0) {
                GN_out = &(*path).second[count];
                 break;
            }
        }
         path_point* pro_out=nullptr;
        if (n < (pro_size - 1)&& (*GNs)[(*pro_path).second[n+ 1].path.target_index].name.last_id != 0 //���ȼ��ߵĳ�Ҫ�����ͷ·
            &&(*GNs)[(*pro_path).second[n + 1].path.target_index].name.sec_id == (*GNs)[(*path).second[k + 1].path.target_index].name.sec_id ) {
            for (uint count = n; count < (*pro_path).second.size(); count++) {
                if ((*pro_path).second[count].GN.name.last_id == 0) {
                    pro_out=&(*pro_path).second[count];
                    break;
                }
            }
            if (GN_point->end_time < point_pro->end_time && GN_out->end_time > point_pro->start_time) {//���ȼ��͵ĳ��Ƚ����ͷ·
               //�ı����ȼ�
                pair<Car_config, pathList>temp = (*paths)[j];
                if (i == ((*paths).size() - 1)) {
                    (*paths).push_back(temp);
                }
                else { (*paths).insert((*paths).begin() + i + 1, temp); }
                (*paths)[j].first.index = -1;
                temp.second.clear();
                (*paths)[j].second.clear();

            }
            else if (GN_point->end_time > point_pro->end_time && GN_out->end_time < point_pro->start_time) {          //���ȼ��ߵĳ��Ƚ����ͷ·
                //�жϳ�վ����,
                if (pro_out->path.target_index == GN_point->GN.index) {   //��ͻ
                    replanning_path(path, GN_point, GNs);
                    
                }

                else if (pro_out->path.target_index != GN_point->index) {//��򵥵����
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
    int* new_satrt_point = nullptr;  //��ͼ���¹滮ʱ����Ҫ��������յ��������
    int* new_end_point = nullptr;
    uint c = 0;
    int* start_point_index = nullptr;   //�ö�·��·��������������
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
    if (c +1== (path->first.middle_point.size()-1)) {  //��Ϊ���һ��·
        is_end_path = true;
    }

    start_point_index = &(path->first.middle_point[c].first);
    target_point_index = &(path->first.middle_point[c + 1].first);

    //ɾ���ϵ�·��
    double start_time = path->second[*start_point_index].start_time;
   (*path).second.erase((*path).second.begin() + (*start_point_index), (*path).second.begin() + (*target_point_index));

    pathList new_way = findPath((*GNs)[*new_satrt_point], (*GNs)[*new_end_point], GNs);
    car_path temp_path((*path).first, new_way);

    //ʱ�䴰��ʼ��

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
     //���¸������������
     for (uint m = i; m < path->second.size();m++) {
         if (m == path->second.size() - 1) { (*path).second[m].index = -1; }
         else {
             (*path).second[m].index = m;
         }
     }

     //���¸���·����ʼ����Ϣ
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
//            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//ɾ����ײ·�ε�Ŀ���
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
//            std::cout << "���¹滮���" << endl;
//        }
//    }
//    return new_way;
//}
