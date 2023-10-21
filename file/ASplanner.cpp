#include "ASplanner.h"
#include <algorithm>
#include <math.h>
#include <iostream>
using namespace std;
using namespace std::placeholders;
#define car_v 0.01;//�ĳ�ȫ�ֱ���
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

ASplanner::uint  ASplanner::Node::getScore()//���ۺ���
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

//�ȹ滮����·�����滮��ֱ�Ӽ���ʱ�䴰��Ȼ�����ʱ�䴰������һ������·��,�滮����ʱ�䴰��ͬʱ����ǰ���ѹ滮����ʱ�䴰
int ASplanner::Generator::A_star_time_window(vector<pathList>* paths, G_Node source_, G_Node target_, vector<G_Node>* GNs, vector<Gdge_property>* GEs)
{
    Node* current = nullptr;//��ǰ�ڵ��ַ
    NodeSet openSet, closedSet;//����openlist��closelist,���ߵ��������Ͷ���Node*���͵�����
    openSet.reserve(100);//Ԥ�����ڴ�
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));//�������Ϣ����openlist��

    while (!openSet.empty()) //openlist��Ϊ���򷵻�True��Ȼ��ȡ��������̽������
    {
        auto current_it = openSet.begin();//current_it��openlist�ĵ�һ���ڵ�ĵ�ַ
        current = *current_it;//current�ǽڵ��е�����

        for (auto it = openSet.begin(); it != openSet.end(); it++) {//it��openlist��һ����Ա��ʼ������һֱ��openlist��β
            auto node = *it;//ȡ��it������
            if (node->state == 1)//����㲻����
            {
                if (node->getScore() <= current->getScore()) {//���openlist�ڵ�Ĵ���С�ڵ�ǰ�ڵ㣬�ǽ���ǰ�ڵ����
                    current = node;//�ı䵱ǰ�ڵ��ַ
                    current_it = it;//������һ���ڵ�
                }
            }
        }

        if (current->coordinates == target_.coordinates) {//�ҵ�Ŀ��ڵ����˳�
            break;
        }

        closedSet.push_back(current);//�������Χ������С�ĵ����closelist
        openSet.erase(current_it);//����current_it

        for (auto edge : current->GN.link_edges)//Ѱ�ҵ�ǰ������ڱ�
        {
            if (findNodeOnList(closedSet, edge.target_index))
            {//�Ѿ��������ı�����������ֹ�㷵�ظ��ڵ�
                continue;
            }


            uint totalCost = current->G + edge.leng;//·�γ��ȴ������

            Node* successor = findNodeOnList(openSet, edge.target_index);//�Ƿ��ں�ѡ·�������ڣ���Ӵ��µ㣻

            if (successor == nullptr)
            {//̽���µ�

                successor = new Node((*GNs)[edge.target_index], current);//ʵ�������󣬱�֤ռ�ÿռ�
                if (successor->GN.state == 0)
                {
                    delete successor;
                    continue;
                }
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_.coordinates);//�������
                openSet.push_back(successor);
                delete successor;
            }
            else if (totalCost < successor->G)
            {//��������ҵ��ĵ���ۻ�С
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

    G_NodeList path;//G_Node����
    pathList time_path;//path_point����
    while (current != nullptr) //������ѯ
    {
        path.push_back(current->GN);
        current = current->parent;
    }
    reverse(path.begin(), path.end());//����
    for (uint i = 0; i < path.size(); i++)
    {
        path_point temp_t;
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
        time_path.push_back(temp_t);
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return 0;
}

int  ASplanner::Generator::time_window(vector<pathList>* paths, vector<G_Node>* GNs, vector<Gdge_property>* GEs)//paths��A*�㷨�õ�������agv·��,���ڹ滮ȫ��ʱ�䴰
{
    double time_cnt = 0;
    for (auto& path : *paths) {//�ֻ�ʱ�䴰,path��ÿ��agv��·����Ϣ
        time_cnt = 0;
        for (auto& GN_point : path)
        {//����ת�������������������ô��������
            GN_point.start_time = time_cnt;
            if (GN_point.index == 0) {
                GN_point.spend_time = GN_point.path.leng / 100 + 2;
            }
            else if (GN_point.index == -1) {//���һ���ڵ�
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
    for (uint i = 0; i < (*paths).size(); i++) {//i��ָ��i��С��
        if (i == 0)
        {
            continue;
        }
        for (uint k = 0; k < ((*paths)[i]).size(); k++)//�ȶԣ���������,k�ǵ�һ��agv����·����ÿ���м��
        {
            auto GN_point = &((*paths)[i][k]);//GN_point�ǵ�i��С���ĵ�k���м��
            for (uint j = 0; j < i; j++)//��j��С��
            {
                uint l = 0;//��ʾ��j��С���ĵ�l��·��
                for (auto& point_pro : (*paths)[j])//������j��С����·������point_pro����ʾ�ó����м�ڵ�
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

                            double set_time = point_pro.end_time - (*paths)[i][k].start_time;//�������ʱ��
                            for (uint m = k; m < ((*paths)[i]).size(); m++)//�����ĵ�ȫ�����
                            {
                                (*paths)[i][m].start_time += set_time; //����ʱ�䴰 
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
                    else if(GN_point->path.source_index == point_pro.path.target_index && GN_point->path.target_index == point_pro.path.source_index)//���������ͻ
                    {
                        /*GN_point->GN.state = 0;
                        (*paths)[i] = findPath(GN_point->GN, ((*paths)[i][(*paths)[i].size()-1]).GN, GNs, GEs);
                        time_window_dan(paths, i, GNs, GEs);
                        GN_point->GN.state = 1;*/

                        /*for (uint m = k - 1; m < ((*paths)[i]).size(); m++)//ɾ����i�����ĺ���·��
                        {
                            ((*paths)[i]).vector::pop_back();
                        }*/

                        if ((GN_point->start_time<point_pro.end_time && GN_point->start_time>point_pro.start_time) || (GN_point->end_time<point_pro.end_time && GN_point->end_time>point_pro.start_time) || (GN_point->start_time <= point_pro.start_time && GN_point->end_time >= point_pro.end_time))
                        {
                            static int count_cft_opposite = 0;//��̬�������ǳ�ͻ����
                            string temp;//�����ַ���ƴ�ӵ��м����
                            ofstream ofs;//�ļ���

                            if (count_cft_opposite == 0)
                            {
                                ofs.open("D:\\agv_project\\Project1\\file\\conflict.txt", ios::out);//���ļ�����������Ϊд��ģʽ
                            }
                            else
                            {
                                ofs.open("D:\\agv_project\\Project1\\file\\conflict.txt", ios::app);//���ļ�����������Ϊ����ģʽ
                            }
                            //if (count_cft_opposite == 0)
                            //{
                            //    ofs << setw(3) << setfill(' ') << "High priority" << setw(15) << setfill(' ') << "Direction" << setw(30) << setfill(' ') << "Time";
                            //    ofs << setw(3) << setfill(' ') << "Low priority" << setw(15) << setfill(' ') << "Direction" << setw(30) << setfill(' ') << "Time" << endl;
                            //}
                            //��д���ͬʱ����֤����
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
                        
                        //pathList temp_path;//��סҪ���ĵ�agv��·����
                        // auto temp_path = ASplanner::Generator::findPath(point_pro.GN.source_index, point_pro, *GNs, *GEs);//�ع滮��j������·��
                    }
                    else if (GN_point->path.target_index == point_pro.path.target_index)//�յ���ͬʱ
                    {
                        if(GN_point->end_time == point_pro.end_time)
                            operator_num = 1;

                        if (operator_num == 1)
                        {
                            operator_num = 0;

                            double set_time = 5;//�������ʱ��
                            for (uint m = k; m < ((*paths)[i]).size(); m++)//�����ĵ�ȫ�����
                            {
                                (*paths)[i][m].start_time += set_time; //����ʱ�䴰 
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

int  ASplanner::Generator::time_window_dan(vector<pathList>* paths, int count, vector<G_Node>* GNs, vector<Gdge_property>* GEs)//paths��A*�㷨�õ�������agv·��,���滮����ʱ�䴰
{
    /*duble time_cnt = 0;

    auto path = (*paths)[count]; //�ֻ�ʱ�䴰,path��ÿ��agv��·����Ϣ

    time_cnt = 0;
    for (auto& GN_point : path)
    {//����ת�������������������ô��������
        GN_point.start_time = time_cnt;
        if (GN_point.index == 0) {
            GN_point.spend_time = GN_point.path.leng / 100 + 2;
        }
        else if (GN_point.index == -1) {//���һ���ڵ�
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

    for (uint k = 0; k < ((*paths)[count]).size(); k++)//�ȶԣ���������,k�ǵ�һ��agv����·����ÿ���м��
    {
        auto GN_point = &((*paths)[count][k]);//GN_point�ǵ�i��С���ĵ�k���м��
        for (uint j = 0; j < count; j++)//��j��С��
        {
            uint l = 0;//��ʾ��j��С���ĵ�l��·��
            for (auto& point_pro : (*paths)[j])//������j��С����·������point_pro����ʾ�ó����м�ڵ�
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

                        double set_time = point_pro.end_time - (*paths)[count][k].start_time;//�������ʱ��
                        for (uint m = k; m < ((*paths)[count]).size(); m++)//�����ĵ�ȫ�����
                        {
                            (*paths)[count][m].start_time += set_time; //����ʱ�䴰 
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
                else if (GN_point->path.source_index == point_pro.path.target_index && GN_point->path.target_index == point_pro.path.source_index)//���������ͻ
                {
                    GN_point->GN.state = 0;
                    (*paths)[count] = findPath(GN_point->GN, ((*paths)[count][(*paths)[count].size()-1]).GN, GNs, GEs);
                    time_window_dan(paths, count, GNs, GEs);
                    GN_point->GN.state = 1;
                }
                else if (GN_point->path.target_index == point_pro.path.target_index)//�յ���ͬʱ
                {
                    if (GN_point->end_time == point_pro.end_time)
                        operator_num = 1;

                    if (operator_num == 1)
                    {
                        operator_num = 0;

                        double set_time = 5;//�������ʱ��
                        for (uint m = k; m < ((*paths)[count]).size(); m++)//�����ĵ�ȫ�����
                        {
                            (*paths)[count][m].start_time += set_time; //����ʱ�䴰 
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

ASplanner::pathList  ASplanner::Generator::findPath(G_Node source_, G_Node target_, vector<G_Node> *GNs, vector<Gdge_property> *GEs)//һ�ι滮һ������·��
{
    Node* current = nullptr;//��ǰ�ڵ��ַ
    NodeSet openSet, closedSet;//����openlist��closelist,���ߵ��������Ͷ���Node*���͵�����
    openSet.reserve(100);//Ԥ�����ڴ�
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));//�������Ϣ����openlist��

    while (!openSet.empty()) //openilst��Ϊ���򷵻�True��Ȼ��ȡ��������̽������
    {
        auto current_it = openSet.begin();//current_it��openlist�ĵ�һ���ڵ�ĵ�ַ
        current = *current_it;//current�ǽڵ��е�����

        for (auto it = openSet.begin(); it != openSet.end(); it++) {//it��openlist��һ����Ա��ʼ������һֱ��openlist��β
            auto node = *it;//ȡ��it������
            //if (node->state == 1)//����㲻����
            //{
            if (node->getScore() <= current->getScore()) {//���openlist�ڵ�Ĵ���С�ڵ�ǰ�ڵ㣬�ǽ���ǰ�ڵ����
                current = node;//�ı䵱ǰ�ڵ��ַ
                current_it = it;//������һ���ڵ�
            }
            //}
        }

        if (current->coordinates == target_.coordinates) {//�ҵ�Ŀ��ڵ����˳�
            break;
        }

        closedSet.push_back(current);//�������Χ������С�ĵ����closelist
        openSet.erase(current_it);//����current_it

        for (auto edge : current->GN.link_edges)//Ѱ�ҵ�ǰ������ڱ�
        {
            if (findNodeOnList(closedSet, edge.target_index)) 
            {//�Ѿ��������ı�����������ֹ�㷵�ظ��ڵ�
                continue;
            }
     
            uint totalCost = current->G + edge.leng;//·�γ��ȴ������

            Node* successor = findNodeOnList(openSet, edge.target_index);//�Ƿ��ں�ѡ·�������ڣ���Ӵ��µ㣻

            if (successor == nullptr)
            {//̽���µ�
                
                successor = new Node((*GNs)[edge.target_index], current);//ʵ�������󣬱�֤ռ�ÿռ�
                /*if (successor->GN.state == 0)
                {
                    delete successor;
                    continue;
                }*/
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_.coordinates);//�������
                openSet.push_back(successor);
                //delete successor;
            }
            else if (totalCost < successor->G)
            {//��������ҵ��ĵ���ۻ�С
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

    G_NodeList path;//G_Node����
    pathList time_path;//path_point����
    while (current != nullptr) //������ѯ
    {
        path.push_back(current->GN);
        current = current->parent;
    }
    reverse(path.begin(), path.end());//����
    for (uint i = 0; i < path.size(); i++) 
    {
        path_point temp_t;
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
