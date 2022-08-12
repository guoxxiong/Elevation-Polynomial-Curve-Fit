/*BEGIN_FILE_HDR
************************************************************************************
*NOTICE

************************************************************************************
*File Name:create_xml
************************************************************************************
*Project/Product:ROS VSLAM
*Title:polynomial elevation
*Author:xiong.guo
************************************************************************************
*Description:
*
*(Requirements,pseudo code and etc.)
************************************************************************************
*Limitations:
*
*(limitations)
************************************************************************************

************************************************************************************
*Revision History:
*
*Version          Date         Initials        CR#          Descriptions
*--------     -----------     -----------   ---------     ----------------------
*1.0           05/07/18                       N/A            Original
*
************************************************************************************
*END_FILE_HDR*/


/*Includes*/
/*****************************************************************************************/
#include<highgui.h>
#include<vector>
#include<cmath>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <libxml/tree.h>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#define DEFAULT_XML_FILE "generated_true.xodr"
#define SN 1000
#define SSN 100



using namespace std;
using namespace cv;


/*Static variables*/
/*****************************************************************************************/
typedef pcl::PointXYZRGB PointT;
std::vector< vector<PointT> > PointsVector;
std::vector<PointT> PointVector;
PointT p;
//double s[SSN][SN];
std::vector< std::vector<double> > ss;
std::vector<double> s;
double sv = 0;
double a[SSN][SN],b[SSN][SN],c[SSN][SN],d[SSN][SN];
int s_count = 0;
int ss_count = 0;


/*BEGIN_FUNCTION_HDR
*******************************************************************************************
*Function Name:polynomialCurveFit()
*Description:给定几个点，求出多项式拟合曲线的系数矩阵Ａ
*
*
*Inputs:vector<Point> , n
*
*
*Outputs:A
*
*
*Limitations:
*******************************************************************************************
END_FUNCTION_HDR*/
bool polynomialCurveFit(vector<Point> & training,int n,Mat& A)
{
    int N = training.size();
    Mat x=Mat::zeros(n+1,n+1,CV_64FC1);
    for (int i = 0;i < n+1;i++)
    {
        for (int j = 0;j < n+1;j++)
        {
            for (int k = 0;k < N;k++)
            {
                    x.at<double>(i,j) =  x.at<double>(i,j) + pow(training[k].x,i+j);
            }
        }
    }
    Mat Y = Mat::zeros(n+1,1,CV_64FC1);
    for (int i = 0;i < n+1;i++)
    {
        for (int k = 0;k < N;k++)
        {
            Y.at<double>(i,0) = Y.at<double>(i,0) + pow(training[k].x,i)*training[k].y;
        }
    }
    A = Mat::zeros(n+1,1,CV_64FC1);
    solve(x,Y,A,cv::DECOMP_LU);
    return true;
}


/*BEGIN_FUNCTION_HDR
*******************************************************************************************
*Function Name:node_point_init
*Description:将字符串char str[]转换为三维坐标存储到PointVector
*
*
*Inputs:str_to_point()
*
*
*Outputs:void
*
*
*Limitations:
*******************************************************************************************
END_FUNCTION_HDR*/
void str_to_point(char str[])
{
    int i=0;
    int j=0;
    while(str[i]!='\0')
    {
        while((str[i]!=' ')&&str[i]!='\0')
        {
            for(int l=0;l<3;l++)
            {
                char str1[20]="\0";
                int leng=0;
                float x1,y1,z1;
                while((str[i]!=',')&&(str[i]!=' ')&&(str[i]!='\0'))
                {
                    str1[leng]=str[i];
                    i++;
                    leng++;
                }
                if(l==0)
                { 
                    sscanf(str1,"%f",&x1);
                    p.x=x1;
                    
                }
                if(l==1)
                { 
                    sscanf(str1,"%f",&y1);
                    p.y=y1;
                }
                if(l==2)
                { 
                    sscanf(str1,"%f",&z1);
                    p.z=z1;
                } 
                i++;  
           }
           PointVector.push_back(p);
           if((str[i-1]!='\0')&&(str[i-1]!=' '))
           { i++; }
           if(str[i-1]=='\0')
           { i--; }
               
        }
    }
}


/*BEGIN_FUNCTION_HDR
*******************************************************************************************
*Function Name:main
*Description:fitting road elevations
*
*
*Inputs:
*
*
*Outputs:
*
*
*Limitations:
*******************************************************************************************
END_FUNCTION_HDR*/



/*BEGIN_FUNCTION_HDR
*******************************************************************************************
*Function Name:parse_xodr_s()
*Description:get the "s" property of geometry from a xml file
*
*
*Inputs:FILE_NAME 
*
*
*Outputs:void
*
*
*Limitations:
*******************************************************************************************
END_FUNCTION_HDR*/
static int parse_xodr_s(const char *file_name)
{
    assert(file_name);
    xmlKeepBlanksDefault(0);  //不设置会影响光标的跳动（需要两次才能跳出一个节点）

    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr roadcur;//定义一个指向road节点的指针

    doc = xmlParseFile(file_name);
        
    if (doc == NULL)
    {
        fprintf(stderr,"Failed to parse xml file:%s\n",file_name);
        goto FAILED;
    }

    roadcur = xmlDocGetRootElement(doc);
    
    if (roadcur == NULL)
    {
        fprintf(stderr,"Root is empty.\n");
        goto FAILED;
    }

    if ((xmlStrcmp(roadcur->name,(const xmlChar *)"OpenDRIVE")))
    {
        fprintf(stderr,"The root is not OpenDRIVE.\n");
        goto FAILED;
    }
    
    roadcur = roadcur->xmlChildrenNode;  //光标跳到header节点
    //printf("%s",roadcur->name);
    roadcur = roadcur->next;   //光标跳到road节点
    while ((roadcur != NULL) && (!xmlStrcmp(roadcur->name,(const xmlChar *)"road")))
    {
        cur = roadcur;
        xmlChar *isJunction;
        isJunction = xmlGetProp(cur,BAD_CAST "junction");
        printf("junction=%s\n",isJunction);
        if ((!xmlStrcmp(isJunction,(const xmlChar*) "-1")))
        {
            cur = cur->xmlChildrenNode;//跳至road子节点
            while ((xmlStrcmp(cur->name,(const xmlChar *)"planView")))
            {
                cur = cur->next;
            }
            //s_count = 0;
            
            //printf("%s",cur->name);
            /*cur = cur->xmlChildrenNode;//跳至road节点的子节点type
            printf("%s",cur->name);
            cur = cur->next; //planView
            cur = cur->xmlChildrenNode; //geometry
            printf("%s",cur->name);*/

            /*xmlChar *ps;
            ps=xmlGetProp(cur,BAD_CAST "s");
            printf("%s",ps);
            memset(ps,'\0',sizeof(ps));
            cur=cur->next;
            ps=xmlGetProp(cur,BAD_CAST "s");
            printf("%s",ps);*/
       
            while (cur != NULL)
            {
                if ((!xmlStrcmp(cur->name,(const xmlChar*) "planView")))
                {
                    cur = cur->xmlChildrenNode;//跳至geometry节点
                    xmlChar *s_str;  //用来存储geometry节点下s属性的值
                    while ((!xmlStrcmp(cur->name,(const xmlChar*) "geometry")))
                    {
                        s_str = xmlGetProp(cur,BAD_CAST "s");
                        //std::cout<<s_str<<std::endl;
                        sscanf((char*)s_str,"%lf",&sv);
                        memset(s_str,'\0',sizeof(s_str));
                        s.push_back(sv);
                        if(cur->next!=NULL)
                        {
                            cur = cur->next;
                        }
                        else
                            break;
                    }
                    cur = NULL;
                }
                else
                {
                    cur = cur->next;
                }
            }
        }
        if ((!xmlStrcmp(roadcur->name,(const xmlChar *)"road")) && (!xmlStrcmp(isJunction,(const xmlChar *)"-1")))
        {
            xmlChar *length_str;
            length_str = xmlGetProp(roadcur,BAD_CAST "length");
            sscanf((char*)length_str,"%lf",&sv);
            s.push_back(sv);
            ss.push_back(s);
        }
        s.clear();
        roadcur = roadcur->next;
    }
    xmlFreeDoc(doc);
    return 1;

FAILED:
    if (doc)
    {
        xmlFreeDoc(doc);
        return -1;
    }
}


//将拟合好的高度曲线方程写到OpenDRIVE
static int fix_xodr_elevation(const char *file_name,std::vector< std::vector<double> > SS,double A[SSN][SN],double B[SSN][SN],double C[SSN][SN],double D[SSN][SN])
{
    assert(file_name);
    xmlKeepBlanksDefault(0);  //不设置会影响光标的跳动（需要两次才能跳出一个节点）

    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr roadcur;//定义一个指向road节点的指针
    xmlNodePtr elevationProfilecur;//定义指向elevationProfile节点的xml指针
    xmlChar *s;
    xmlChar *a;
    xmlChar *b;
    xmlChar *c;
    xmlChar *d;
    int SS_count = 0;
    int S_count = 0;
    int aa_count = 0;
    int a_count = 0;
    int bb_count = 0;
    int b_count = 0;
    int cc_count = 0;
    int c_count = 0;
    int dd_count = 0;
    int d_count = 0;
    double s_double;

    doc = xmlParseFile(file_name);
        
    if (doc == NULL)
    {
        fprintf(stderr,"Failed to load xml file:%s\n",file_name);
        goto FAILED;
    }

    roadcur = xmlDocGetRootElement(doc);
    
    if (roadcur == NULL)
    {
        fprintf(stderr,"Root is empty.\n");
        goto FAILED;
    }

    if ((xmlStrcmp(roadcur->name,(const xmlChar *)"OpenDRIVE")))
    {
        fprintf(stderr,"The root is not OpenDRIVE.\n");
        goto FAILED;
    }
    
    roadcur = roadcur->xmlChildrenNode;  //光标跳到header节点
    //printf("%s",roadcur->name);
    roadcur = roadcur->next;   //光标跳到road节点
    while ((roadcur != NULL) && (!xmlStrcmp(roadcur->name,(const xmlChar *)"road")))
    {
        cur = roadcur;
        cur = cur->next;  //光标跳到link
        cur = cur->next;  //光标跳到type
        cur = cur->next;  //光标跳到planView
        cur = cur->next;  //光标跳到elevationProfile
        elevationProfilecur = cur;
        cur = cur->xmlChildrenNode;//跳到elevation
        s_double = SS[SS_count][S_count];
        sprintf((char*)s,"%lf",s_double);
        S_count++;
        sprintf((char*)a,"%lf",A[aa_count][a_count]);
        a_count++;
        sprintf((char*)b,"%lf",A[bb_count][b_count]);
        b_count++;
        sprintf((char*)c,"%lf",A[cc_count][c_count]);
        c_count++;
        sprintf((char*)d,"%lf",A[dd_count][d_count]);
        d_count++;
        xmlNewProp(cur,BAD_CAST "s",s);
        xmlNewProp(cur,BAD_CAST "a",a);
        xmlNewProp(cur,BAD_CAST "b",b);
        xmlNewProp(cur,BAD_CAST "c",c);
        xmlNewProp(cur,BAD_CAST "d",d);

        for (int i = 1;i < SS[SS_count].size();i++)
        {
            s_double = SS[SS_count][S_count];
            sprintf((char*)s,"%lf",s_double);
            S_count++;
            sprintf((char*)a,"%lf",A[aa_count][a_count]);
            a_count++;
            sprintf((char*)b,"%lf",A[bb_count][b_count]);
            b_count++;
            sprintf((char*)c,"%lf",A[cc_count][c_count]);
            c_count++;
            sprintf((char*)d,"%lf",A[dd_count][d_count]);
            d_count++;
            xmlNodePtr new_node = xmlNewNode(NULL,BAD_CAST "elevation");
            xmlAddChild(elevationProfilecur,new_node);
            cur = cur->next;
            xmlNewProp(cur,BAD_CAST "s",s);
            xmlNewProp(cur,BAD_CAST "a",a);
            xmlNewProp(cur,BAD_CAST "b",b);
            xmlNewProp(cur,BAD_CAST "c",c);
            xmlNewProp(cur,BAD_CAST "d",d);
        }
        SS_count++;
        
    }
    FAILED:
    if (doc)
    {
        xmlFreeDoc(doc);
        return -1;
    }
    
}


int main()
{
    //从opendrive文件中提取s
    int f;
    const char *xml_file = DEFAULT_XML_FILE;
    f=parse_xodr_s(xml_file);
    printf("s:\n");
    printf("ss_size:%lu\n",ss.size());
    for (int i = 0;i < ss.size();i++)
    {
        for (int j = 0;j < ss[i].size();j++)
        {
            cout<<ss[i][j]<<" ";
        }
        printf("\n");
    }

    printf("f=%d\n",f);
    
    std::string filename = "data.txt";  //从一个txt文档中读取坐标
    std::ifstream infile;
    infile.open(filename.data());
    assert(infile.is_open());
    std::string read_s;
    while (getline(infile,read_s))
    {
        std::cout<<read_s<<std::endl;
        char strs[read_s.length()];
        strcpy(strs,read_s.c_str());
        str_to_point(strs);
        PointsVector.push_back(PointVector);
        PointVector.clear();    
    }
    infile.close();

    Mat img = Mat::zeros(480,640,CV_8UC3);
    img.setTo(Scalar(0,0,0));
    //char strs1[]="1.90,0.63,0.40 9.41,3.43,2.00 16.98,6.03,3.60 25.98,7.47,5.30 34.02,7.23,6.46 58.49,2.65,8.00 78.78,0.00,6.94 89.92,0.57,5.82 99.89,1.70,4.88 115.83,4.40,3.80 131.76,7.81,2.70 143.70,10.56,1.88 157.63,13.70,0.93 165.59,15.35,0.38 171.18,16.40,0.00";
    //str_to_point(strs1);//将上述字符串转换为坐标点
    vector<Point> points;
    for (int i=0;i < PointsVector.size();i++)
    {
        for (int j = 0;j < PointsVector[i].size()-1;j++)
        {
            if (j == 0) //先利用4个点拟合出一条线的方程作为后续求解的参考
            {
                // points.push_back(Point(ss[i][j],PointsVector[i][j].z));
                // points.push_back(Point(ss[i][j+1],PointsVector[i][j+1].z));
                // points.push_back(Point(ss[i][j+2],PointsVector[i][j+2].z));
                // points.push_back(Point(ss[i][j+3],PointsVector[i][j+3].z));
                // Mat A;
                // polynomialCurveFit(points,3,A); //高度拟合
                a[i][j] = PointsVector[i][j].z;
                b[i][j] = (PointsVector[i][j+1].z-PointsVector[i][j].z)/(ss[i][j+1]-ss[i][j]);
                c[i][j] = 0;
                d[i][j] = 0;
            }
            else 
            {
                a[i][j] = a[i][j-1] + b[i][j-1]*(ss[i][j]-ss[i][j-1]) + c[i][j-1]*pow((ss[i][j]-ss[i][j-1]),2) + d[i][j-1]*pow((ss[i][j]-ss[i][j-1]),3);//根据a为上一个entry的出口的高度计算a[i]
                b[i][j] = b[i][j-1] + 2*c[i][j-1]*(ss[i][j]-ss[i][j-1]) + 3*d[i][j-1]*pow((ss[i][j]-ss[i][j-1]),2);//根据拟合出的曲线要使有点的地方的两entry的斜率相等(使下一个entry能与上一个entry的斜率衔接上)

                Mat C = Mat::zeros(2,2,CV_64FC1);
                C.at<double>(0,0) = pow((ss[i][j+1]-ss[i][j]),2);
                C.at<double>(0,1) = pow((ss[i][j+1]-ss[i][j]),3);
                C.at<double>(1,0) = pow((ss[i][j+2]-ss[i][j]),2);
                C.at<double>(1,1) = pow((ss[i][j+2]-ss[i][j]),3);

                Mat E = Mat::zeros(2,1,CV_64FC1);
                E.at<double>(0,0) = PointsVector[i][j+1].z - a[i][j] - b[i][j]*(ss[i][j+1]-ss[i][j]);
                E.at<double>(1,0) = PointsVector[i][j+2].z - a[i][j] - b[i][j]*(ss[i][j+2]-ss[i][j]);

                Mat D = Mat::zeros(2,1,CV_64FC1);
                solve(C,E,D,cv::DECOMP_LU);

                c[i][j] = D.at<double>(0,0);
                d[i][j] = D.at<double>(1,0);
            }
        }
    }

    for (int i = 0;i < PointsVector.size();i++)
    {
        for (int j = 0;j < PointsVector[i].size()-1;j++)
        {
            std::cout<<"<elevation s=\""<<ss[i][j]<<"\""<<" "<<"a=\""<<a[i][j]<<"\""<<" "<<"b=\""<<b[i][j]<<"\""<<" "<<"c=\""<<c[i][j]<<"\" "<<"d=\""<<d[i][j]<<"\""<<"/>"<<std::endl;
            //std::cout<<"slope:"<<b[i][j]+2*c[i][j]*(ss[i][j]-ss[i][j-1])+3*d[i][j]*(ss[i][j]-ss[i][j-1])*(ss[i][j]-ss[i][j-1])<<std::endl;
        }
        printf("\n");
    }

   

    //points.push_back(Point(300.,220.));
    //points.push_back(Point(350.,400.));   
    /*for (int i = 0;i < points.size();i++)
    {
        circle(img,points[i],5,Scalar(0,0,255),2,8,0);
    }
    polylines(img,points,false,Scalar(0,255,0),1,8,0);
    //Mat A;
    //polynomialCurveFit(points,3,A);
    /*a = A.at<double>(0,0);
    b = A.at<double>(1,0);
    c = A.at<double>(2,0);
    d = A.at<double>(3,0);
    cout<<A<<endl;
    cout<<a<<","<<b<<","<<c<<","<<d<<endl;
    vector<Point>points_fitted;
    for (int x = 0;x < 400;x++)
    {
        double y=A.at<double>(0,0) + A.at<double>(1,0)*x + A.at<double>(2,0)*pow(x,2) + A.at<double>(3,0)*pow(x,3);
        points_fitted.push_back(Point(x,y));
    }
    polylines(img,points_fitted,false,cv::Scalar(0,255,255),1,8,0);
    imshow("img",img);
    /*for (int i = 0;i < points.size();i++)
    {
        cout<<points[i].x<<","<<points[i].y<<endl;
    }
    waitKey(0);*/
    return 0;
    
}