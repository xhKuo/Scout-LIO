/****************************
 * 题目：给定一组世界坐标系下的3D点(p3d.txt)以及它在相机中对应的坐标(p2d.txt)，以及相机的内参矩阵。
 * 使用bundle adjustment 方法（g2o库实现）来估计相机的位姿T。初始位姿T为单位矩阵。
 *
* 本程序学习目标：
 * 熟悉g2o库编写流程，熟悉顶点定义方法。
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.02
****************************/

#include <vector>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace Eigen;

using namespace cv;
using namespace std;


string p3d_file = "/home/lab-307/LIO-SAM_ws/src/Scout-LIO/test/data/p3d.txt";
string p2d_file = "/home/lab-307/LIO-SAM_ws/src/Scout-LIO/test/data/p2d.txt";


//// 自定义顶点，6DOF的姿态
class myVertex : public g2o::BaseVertex<6, g2o::SE3Quat>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    myVertex() = default;
    bool read(std::istream& is) override{};
    bool write(std::ostream& os) const override{};
    void setToOriginImpl() override {
        //// 初始值设置为 SE3单位矩阵
        _estimate = g2o::SE3Quat();
    }

    void oplusImpl(const number_t* update_) override {
        Eigen::Map<const g2o::Vector6> update(update_);
        setEstimate(g2o::SE3Quat::exp(update)*estimate());        //更新方式
    }
};


//// 自定义边
class myEdge: public g2o::BaseBinaryEdge<2, g2o::Vector2 , g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    myEdge() = default;
    bool read(istream& in) override{};
    bool write(ostream& out) const override{};
    void computeError() override
    {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const g2o::CameraParameters * cam
                = static_cast<const g2o::CameraParameters *>(parameter(0));
        g2o::Vector2 obs(_measurement);
        _error = obs-cam->cam_map(v1->estimate().map(v2->estimate()));
    }

    void linearizeOplus() override
    {
        g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
        g2o::Vector3 xyz = vi->estimate();
        g2o::Vector3 xyz_trans = T.map(xyz);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z*z;

        const g2o::CameraParameters * cam = static_cast<const g2o::CameraParameters *>(parameter(0));

        Matrix<double,2,3,Eigen::ColMajor> tmp;
        tmp(0,0) = cam->focal_length;
        tmp(0,1) = 0;
        tmp(0,2) = -x/z*cam->focal_length;

        tmp(1,0) = 0;
        tmp(1,1) = cam->focal_length;
        tmp(1,2) = -y/z*cam->focal_length;

        _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

        _jacobianOplusXj(0,0) =  x*y/z_2 *cam->focal_length;
        _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *cam->focal_length;
        _jacobianOplusXj(0,2) = y/z *cam->focal_length;
        _jacobianOplusXj(0,3) = -1./z *cam->focal_length;
        _jacobianOplusXj(0,4) = 0;
        _jacobianOplusXj(0,5) = x/z_2 *cam->focal_length;

        _jacobianOplusXj(1,0) = (1+y*y/z_2) *cam->focal_length;
        _jacobianOplusXj(1,1) = -x*y/z_2 *cam->focal_length;
        _jacobianOplusXj(1,2) = -x/z *cam->focal_length;
        _jacobianOplusXj(1,3) = 0;
        _jacobianOplusXj(1,4) = -1./z *cam->focal_length;
        _jacobianOplusXj(1,5) = y/z_2 *cam->focal_length;
    }
private:
    // data
};


void bundleAdjustment (
        const vector<Point3f> points_3d,
        const vector<Point2f> points_2d,
        Mat& K,
        bool useDefaultEdge
        );


int main(int argc, char **argv) {

    bool useDefaultEdge = true;
    vector< Point3f > p3d;
    vector< Point2f > p2d;

    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    // 导入3D点和对应的2D点

    ifstream fp3d(p3d_file);
    if (!fp3d){
        cout<< "No p3d.text file" << endl;
        return -1;
    }
    else {
        while (!fp3d.eof()){
            double pt3[3] = {0};
            for (auto &p:pt3) {
                fp3d >> p;
            }
            p3d.push_back(Point3f(pt3[0],pt3[1],pt3[2]));
        }
    }
    ifstream fp2d(p2d_file);
    if (!fp2d){
        cout<< "No p2d.text file" << endl;
        return -1;
    }
    else {
        while (!fp2d.eof()){
            double pt2[2] = {0};
            for (auto &p:pt2) {
                fp2d >> p;
            }
            Point2f p2(pt2[0],pt2[1]);
            p2d.push_back(p2);
        }
    }

    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    bundleAdjustment ( p3d, p2d, K , useDefaultEdge);
    return 0;
}


void bundleAdjustment (
        const vector< Point3f > points_3d,
        const vector< Point2f > points_2d,
        Mat& K,
        bool useDefaultEdge )
{
    // creat g2o
    // new g2o version. Ref:https://www.cnblogs.com/xueyuanaichiyu/p/7921382.html

    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    // 第1步：创建一个线性求解器LinearSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();

    // 第2步：创建 BlockSolver。并用上面定义的线性求解器初始化
    Block* solver_ptr = new Block (  std::unique_ptr<Block::LinearSolverType>(linearSolver) );

    // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<Block>(solver_ptr) );

    // 第4步：创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose(true);
//    // old g2o version
//    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
//    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
//    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
//    g2o::SparseOptimizer optimizer;
//    optimizer.setAlgorithm ( solver );

    // 第5步：定义图的顶点和边。并添加到SparseOptimizer中

    // ----------------------开始你的代码：设置并添加顶点，初始位姿为单位矩阵

//    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
//    pose->setId(0);
//    pose->setEstimate(g2o::SE3Quat());
//    optimizer.addVertex(pose);
    myVertex *pose = new myVertex();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat());
    optimizer.addVertex(pose);

    int index = 1;
    for ( const Point3f p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true );
        optimizer.addVertex ( point );
    }
    // ----------------------结束你的代码

    // 设置相机内参
    g2o::CameraParameters* camera = new g2o::CameraParameters (
            K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0);
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // 设置边
    if (useDefaultEdge){
        index = 1;
        for ( const Point2f p:points_2d )
        {
            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId ( index );
            edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
            edge->setVertex ( 1, pose );
            edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );  //设置观测值
            edge->setParameterId ( 0,0 );
            edge->setInformation ( Eigen::Matrix2d::Identity() );
            optimizer.addEdge ( edge );
            index++;
        }
    } else {
        index = 1;
        for( const Point2f p:points_2d )
        {
            myEdge *XYZ2UV = new myEdge();
            XYZ2UV->setId(index);
            XYZ2UV->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
            XYZ2UV->setVertex ( 1, pose );
            XYZ2UV->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );  //设置观测值
            XYZ2UV->setParameterId ( 0,0 );
            XYZ2UV->setInformation ( Eigen::Matrix2d::Identity() );
            optimizer.addEdge ( XYZ2UV );
            index++;
        }
    }


    // 第6步：设置优化参数，开始执行优化
    optimizer.setVerbose ( false );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );

    // 输出优化结果
    cout<< endl <<"after optimization:"<<endl;
    cout<< "T=" << endl << Eigen::Isometry3d ( pose->estimate() ).matrix() << endl;
}

