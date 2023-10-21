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
    //��ʼ���ݸ���·�����Ⱥͳ����ٶ�������û�п�����ײ��ʱ�䴰��ÿһ�εĿ�ʼʱ��ͽ���ʱ�䣩
    for (auto& path : *paths) {//�ֻ�ʱ�䴰
        time_cnt = 0;
        for (auto& GN_point : path)
        {
            GN_point.start_time = time_cnt;
            if (GN_point.index == 0) {      //0�Ļ����ǵ�һ���ڵ㣬��������ʱ��
                GN_point.spend_time = GN_point.path.leng / 100 + 2;   //???
            }
            else if (GN_point.index == -1) {    //-1�Ļ�������û�нڵ�
                GN_point.spend_time = 0;
            }
            //ת�䡢
            //ֱ����
            //����
            //����
            else {                  //�����Ļ�����������·�ε�ʱ��
                GN_point.spend_time = GN_point.path.leng / car_v;//����ת��
            }
            time_cnt += GN_point.spend_time;
            GN_point.end_time = time_cnt;
        }
    }
    uint path_cnt = 0;
    uint operator_num = 0;//1Ϊǰ���ú󳵣�2Ϊ����ǰ������������
    for (uint i = 0; i < (*paths).size(); i++) {    // iС������
        if (i == 0)
        {
            continue;
        }
        //����·���ĳ�ͻ������ʱ�䴰�ĳ�ͻ����Ϊ������
        for (uint k = 0; k < ((*paths)[i]).size(); k++)//�ȶԣ��������㡣k:��i��С����·���ڵ�λ��
        {
            auto GN_point = &((*paths)[i][k]);//i���ĵ�k���ڵ�
            //auto GN_pointg = &((*paths)[i][k-1]);
            for (uint j = 0; j < i; j++)
            {
                uint size = (*paths)[j].size();
                for (auto& point_pro : (*paths)[j])//���ȼ����ߵĹ켣��
                {
                    //if ((*paths)[i][k].GN.name.main_id == point_pro.GN.name.main_id);
                    //�󳵵�Դ�ڵ����ǰ����Դ�ڵ� ���� �󳵵�Ŀ��ڵ����ǰ����Ŀ��ڵ�
                    if (GN_point->path.source_index == point_pro.path.source_index && GN_point->path.target_index == point_pro.path.target_index)
                    {
                        /*[        ]
                                [        ]

                            [               ]
                                [        ]
                        */
                        //�󳵵Ŀ�ʼʱ��  ����ǰ���Ľ���ʱ�� && ����ǰ���Ŀ�ʼʱ��
                        if (GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time)
                        {
                            operator_num = 1;
                        }
                        /*     [        ]
                          [        ]   ->
                        */
                        //�󳵵Ľ���ʱ��  ����ǰ���Ľ���ʱ�� && ����ǰ���Ŀ�ʼʱ��
                        else if (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time)
                        {
                            operator_num = 1;
                        }
                        /*     [        ]
                           [                ]
                        */
                        //�󳵵Ŀ�ʼʱ������ǰ���Ŀ�ʼʱ�� && �󳵵Ľ���ʱ����������ǰ���Ľ���ʱ��
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

                            double set_time = point_pro.end_time - (*paths)[i][k].start_time;//�������ʱ��
                            //���ݲ�����������ʱ�䴰
                            for (uint m = k; m < ((*paths)[i]).size(); m++)//�����ĵ�ȫ�����
                            {
                                (*paths)[i][m].start_time += set_time; //����ʱ�䴰 
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
                        && GN_point->index != 0  //�󳵲�Ϊ���
                        && point_pro.index != -1) //��ײ·�β����յ�·�� �Ҳ�Ϊ�󳵵����·��
                    {
                        if ((GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time) || (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time))
                        {
                            auto GN_pointg = &((*paths)[i][k - 1]);
                            auto pro_next = &((*paths)[j][point_pro.index + 2]);
                            if (GN_pointg->GN.index != pro_next->GN.index) {//��ײ·�κ������߲�ͬ·
                                // ��ʱ�䴰����
                                /*cout << "�󳵺�һ����" << GN_pointg->GN.name.sec_id << "-" << GN_pointg->GN.name.last_id << endl
                                    << "ǰ����һ����" << pro_next->GN.name.sec_id << "-" << pro_next->GN.name.last_id << endl
                                    << "ǰ��de��" << point_pro.GN.name.sec_id << "-" << point_pro.GN.name.last_id << endl
                                    << "��de��" << GN_point->GN.name.sec_id << "-" << GN_point->GN.name.last_id << endl;*/
                                cout << "����·ִ�еȴ�" << endl;
                                double set_time = pro_next->end_time - (*paths)[i][k].start_time;//�������ʱ��

                                for (uint m = k - 1; m < ((*paths)[i]).size(); m++)//�����ĵ�ȫ�����
                                {
                                    (*paths)[i][m].start_time += set_time; //����ʱ�䴰 
                                    (*paths)[i][m].end_time += set_time;
                                }
                            }
                        }
                    }
                    if (GN_point->path.source_index == point_pro.path.target_index
                        && GN_point->path.target_index == point_pro.path.source_index
                        && GN_point->index != 0)//GN����Ϊ��㣬�����޷����¹滮·��
                    {
                        if ((GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time) || (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time))
                        {
                            int a = (*paths)[i].size();
                            int index = (*paths)[i][a - 1].GN.index;//β��
                            auto GN_pointg = &((*paths)[i][k - 1]);
                            for (auto it = (*GNs).begin(); it != (*GNs).end(); it++)//ɾ����ײ·�ε�Ŀ���
                            {
                                if (it->index == GN_pointg->GN.index)//�ҵ���һ����
                                {           //������һ��������ӹ�ϵ
                                    //cout << GN_pointg->GN.name.main_id << "-" << GN_pointg->GN.name.sec_id<< "-" << GN_pointg->GN.name.last_id << "�����ӹ�ϵ����" << GN_pointg->GN.link_edges.size() << endl;
                                    cout << GN_pointg->GN.name.main_id << "-" << it->name.sec_id << "-" << it->name.last_id << "�����ӹ�ϵ����" << it->link_edges.size() << endl;
                                    for (int tt = 0; tt < it->link_edges.size(); tt++)
                                    {
                                        //cout << "Ŀ���ceshi" << it->link_edges[t].target_index << endl;
                                        if (it->link_edges[tt].target_index == GN_point->path.source_index)//����ϸ���Ŀ���Ϊ�õ㣬ɾ������·��
                                        {
                                            it->link_edges[tt].state = false;
                                        }
                                    }
                                }
                            }
                            pathList xu = findPath((*GNs)[GN_pointg->path.source_index], (*GNs)[index], GNs);
                            cout << "���¹滮���·����" << endl;
                            for (auto t = xu.begin(); t != xu.end(); t++)
                            {
                                cout << t->GN.name.main_id << "-" << t->GN.name.sec_id << "-" << t->GN.name.last_id << endl;
                            }
                            //���ʱ�䴰
                           //ɾ������·��                         
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
                                {      //0�Ļ����ǵ�һ���ڵ㣬��������ʱ��
                                    GN_pointg.spend_time = GN_pointg.path.leng / 100 + 2;
                                }
                                else if (GN_pointg.index == -1) {    //-1�Ļ�������û�нڵ�
                                    GN_pointg.spend_time = 0;
                                }
                                else {                  //�����Ļ�����������·�ε�ʱ��
                                    GN_pointg.spend_time = GN_pointg.path.leng / car_v;//����ת��
                                }
                                time_cntt += GN_pointg.spend_time;
                                GN_pointg.end_time = time_cntt;
                            }
                        }
                    }
                }
            }
        }
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
                    if (fabs(current->GN.coordinates.y - successor->coordinates.y) > 0.5)
                        successor->H = heuristic(successor->coordinates, target_.coordinates) + 6;//������ۣ���ŷʽ����
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
    for (uint i = 0; i < path.size(); i++) 
    {
        path_point temp_t;      //�켣������
        temp_t.GN = path[i];
        if (i + 1 == path.size()) 
        {
            temp_t.index = -1;//�������һ����
        }
        else 
        {
            temp_t.index = i;//�켣������
            for (auto edge : temp_t.GN.link_edges) 
            {
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
