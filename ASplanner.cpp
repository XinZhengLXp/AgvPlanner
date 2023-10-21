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
    for (auto& path : *paths) {//�ֻ�ʱ�䴰
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
    //            && (*paths)[i].second[j].index !=-1)//�õ㲻Ϊ���һ����
    //        {
    //            if ((*paths)[i].second[j+1].GN.name.last_id != 0)//��һ����Ϊ��ͷ·
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
    //cout << "��һ���ȣ�" << (*paths)[0].second.size() << endl;
    for (uint i = 0; i < (*paths).size(); i++) {    // iС������
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
            for (uint k = 0; k < (*paths)[i].second.size() && !flag; k++)//�ȶԣ��������㡣k:��i��С����·���ڵ�λ��
            {
                auto GN_point = &((*paths)[i].second[k]);//i���ĵ�k���ڵ�
                for (uint j = 0; j < i; j++)
                {
                    if (flag)
                    {
                        break;
                    }
                    for (uint n = 0; n < (*paths)[j].second.size();n++)//���ȼ����ߵĹ켣��
                    {
                       // �ڵ��ͻ
                        if ((*paths)[j].second.size() >2 ) {
                            pathList new_node_c_way = node_conflict(&(*paths)[i],k,&(*paths)[j],n, GNs);
                            (*paths)[i].second = new_node_c_way;
                        }

                        //�����ͻ
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
                        //��ͻ���жϼ������
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
                }//��m
            }//��i��k
        }//whileѭ��
        std::cout<<i+1<<" ��" << "whileѭ������ ��" << abc << endl;
        for (auto t = GNs->begin(); t != GNs->end(); t++)//�ָ���ͼ��·��Ϣ
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
    if (time_path[time_path.size()-1].GN.index != target_.index)
    {
        cout << "Ѱ·ʧ�ܣ�" << endl;
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
//�����ͻ
ASplanner::pathList ASplanner::Generator::opposing_conflict(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs)
{
    pathList new_way = {};
    auto point_pro = &(*pro_path).second[n];
    if ((*path).second[k].GN.index == point_pro->path.target_index
        && (*path).second[k].path.target_index == point_pro->path.source_index
        && (*path).second[k].index != 0  //��Ϊ��һ��
        &&  path->second.size() > 2
        )
    {
       // double length_time = 0.0;
        //if (pro_path->first.type == 0) { length_time = agv_length / (*pro_path).first.car_v; }//AGV����/�ٶ�
        //else { length_time = fork_length / (*pro_path).first.car_v; }//�泵����/�ٶ�

        if (((*path).second[k].start_time < point_pro->end_time && (*path).second[k].start_time > point_pro->start_time)
            || ((*path).second[k].end_time < point_pro->end_time  && (*path).second[k].end_time > point_pro->start_time))
        {
            size_t path_size = path->second.size();
            int start_index = ((*path).second[path_size - 1].GN.index);  //���һ��Ԫ�������ŵ�ַ
            int end_index = ((*path).second[0].GN.index);
            auto GN_pointg = &(*path).second[k - 1];

            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//ɾ����ײ·�ε�Ŀ���
            {
                if (it->index == GN_pointg->GN.index)
                {
                    for (int tt = 0; tt < it->link_edges.size(); tt++)
                    {
                        if (it->link_edges[tt].target_index == GN_pointg->path.target_index)//����ϸ���Ŀ���Ϊ�õ㣬ɾ������·��
                        {
                            it->link_edges[tt].state = false;
                        }
                    }
                }
            }
            (*path).second = findPath((*GNs)[start_index], (*GNs)[end_index], GNs);
            
            std::cout << "�����ͻ�滮���" << endl;
            (*path).second = init_time_windows(&(*path), GNs);
            new_way = (*path).second;
        }
    }
    return new_way;
}
//��ͻ��
ASplanner::pathList ASplanner::Generator::collsion_collection(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs)
{
     pathList new_way = {};
     auto point_pro = &(*pro_path).second[n];
     auto GN_point = &(*path).second[k];
    if (GN_point->path.collsion_id == point_pro->path.collsion_id &&GN_point->path.collsion_id != 0
        //������ͬһ·��
        && !(GN_point->path.source_index== point_pro->path.source_index && GN_point->path.target_index == point_pro->path.target_index))
    {
        double length_time = 0.0;
        if (path->first.type == 0) { length_time = agv_length / pro_path->first.car_v; }//AGV����/�ٶ�
        else { length_time = fork_length/ pro_path->first.car_v; }//�泵����/�ٶ�
        
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
                //������ǰ��Ŀ���ȴ�
                for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//ɾ����ײ·�ε�Ŀ���
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
            else {//�ȴ�
                double length_time = 0.0;
                if ((*path).first.type == 0) { length_time = agv_length / (*path).first.car_v; }
                else{ length_time = fork_length / (*path).first.car_v; }
                                      //ǰ��������ٳ���                                   ����ʱ��
                double set_time = fabs(point_pro->end_time - (*path).second[k].start_time)+ length_time;//�������ʱ��

                cout<< "��ͻ��ִ�еȴ�" << endl;
                for (uint m = k; m < (*path).second.size(); m++)//�����ĵ�ȫ�����
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
                        (*path).second[m].start_time += set_time; //����ʱ�䴰 
                        (*path).second[m].end_time += set_time;
                    }
                }
                new_way =(*path).second;
            }
            //��ͻ���ж�
        }
    }
    return new_way;
}
ASplanner::pathList ASplanner::Generator::init_time_windows(pair<Car_config,pathList> *path,vector<G_Node>* GNs)
{
    double time_cnt = 0;
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
            
            if ((*path).first.type==0) 
            {
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
    return (*path).second;
}
//ͬ���ͻ
ASplanner:: car_path ASplanner::Generator::colliding_conflict(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs)
{
    auto GN_point = &((*path).second[k]);
    auto point_pro = &((*pro_path).second[n]);

    if (GN_point->GN.index == point_pro->GN.index &&
        GN_point->path.target_index == point_pro->path.target_index
        //�ڴ˲��������һ��·����ʵ��Ӧ�ò��ᷢ����
         )
    {   //�󳵸�ǰ��
        //if ((*path).second.size() > 2 && k< (*path).second.size()-2)
        //{//С��i�ڴ˶�·ĩβִ�еȴ��ˣ����ȼ��ߵ��Ƿ���ִ�еȴ���ײ��
        //    if ((*path).second[k].end_time < point_pro->end_time  //��ǰ�������
        //        && (*path).second[k].end_time != (*path).second[k + 1].start_time//���������·��βִ�еȴ���
        //        && pro_next->start_time < (*path).second[k + 1].start_time

        //        ) {
        //    }
        //}
        if (GN_point->start_time < point_pro->start_time
            && GN_point->end_time > point_pro->end_time && GN_point->end_time > point_pro->start_time
             ) 
             /* [ ]       ��
            [        ]    ǰ��*/
        {
            //�ҵ��ֲ�·�������ȼ��ߵĳ�����
        }
         else if (GN_point->start_time < point_pro->end_time && GN_point->start_time > point_pro->start_time
             && GN_point->end_time < point_pro->end_time && GN_point->end_time > point_pro->start_time
             ) {
            //��ȡ�󳵼���
            cout <<"����ִ�еȴ���" << (*path).first.car_v << "m/s����Ϊ" << pro_path->first.car_v << "m/s" << endl;
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

//�ڵ��ͻ
ASplanner::pathList ASplanner::Generator::node_conflict(car_path* path,uint k ,car_path* pro_path,uint n ,vector<G_Node>* GNs)
{
    
    if ((*path).second[k].path.source_index == pro_path->second[n].path.target_index //���Ϊǰ����Ŀ���
        && (*path).second[k].path.target_index == pro_path->second[n].path.source_index
        && pro_path->second.size() > 2  //���ڵ�����
        && pro_path->second[n].index < ((*pro_path).second.size() - 2)
        &&path->second.size()>2
        &&path->second[k].index !=0  //��Ϊ��һ���㣬ȷ���к��˿ռ�
        ) //��ײ·�β����յ�·�� �Ҳ�Ϊ�󳵵����·��
    {
        double length_time = 0.0;
        if (pro_path->first.type==0) { length_time = agv_length / pro_path->first.car_v; }//AGV����/�ٶ�
        else { length_time = fork_length / pro_path->first.car_v; }//�泵����/�ٶ�

        if (((*path).second[k].start_time < (*pro_path).second[n].end_time + length_time && (*path).second[k].start_time >(*pro_path).second[n].start_time)
            || ((*path).second[k].end_time < (*pro_path).second[n].end_time + length_time && (*path).second[k].end_time >(*pro_path).second[n].start_time))
        {
            auto pro_next = &(*pro_path).second[(*pro_path).second[n].index+1];
            auto GN_pointg = &(*path).second[k - 1];
            if (GN_pointg->GN.index != pro_next->path.target_index
                &&GN_pointg->path.target_index == pro_next->GN.index) {//��ײ·�κ������߲�ͬ·
                double t = (*pro_path).second[n].end_time - ((((*path).second[k].path.leng / (*pro_path).first.car_v) + fabs((*path).second[k].start_time +(*pro_path).second[n].start_time)) / 2);
                
                double set_time = fabs(pro_next->end_time - (*path).second[k].start_time) + t;//�������ʱ��
                cout <<"�ڵ��ͻִ�еȴ�" << endl;
                for (uint m = k-1; m < (*path).second.size(); m++)//�����ĵ�ȫ�����
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
                        (*path).second[m].start_time += set_time; //����ʱ�䴰 
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
            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//ɾ����ײ·�ε�Ŀ���
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
            std::cout << "���¹滮���" << endl;
        }
    }
    return new_way;
}
