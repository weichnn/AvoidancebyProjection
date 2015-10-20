//
//  main.cpp
//  QP
//
//  Created by WeiChen on 3/28/15.
//  Copyright (c) 2015 WeiChen. All rights reserved.
//

#include <iostream>
#include "Eigen/Dense"
#include "math.h"
#include <vector>
using namespace Eigen;
MatrixXd getPoly(double t)
{
    MatrixXd table(1,10);
    table <<
    1, t, pow(t,2),   pow(t,3),    pow(t,4),    pow(t,5),     pow(t,6),     pow(t,7),      pow(t,8),      pow(t,9);
    return table;
}
MatrixXd getAllTable(double t)
{
    MatrixXd table(5,10);
    table <<
    1., t, pow(t,2),   pow(t,3),    pow(t,4),    pow(t,5),     pow(t,6),     pow(t,7),      pow(t,8),      pow(t,9),
    0., 1.,      2*t, 3*pow(t,2),  4*pow(t,3),  5*pow(t,4),   6*pow(t,5),   7*pow(t,6),    8*pow(t,7),    9*pow(t,8),
    0., 0.,        2.,        6*t, 12*pow(t,2), 20*pow(t,3),  30*pow(t,4),  42*pow(t,5),   56*pow(t,6),   72*pow(t,7),
    0., 0.,        0.,          6.,        24*t, 60*pow(t,2), 120*pow(t,3), 210*pow(t,4),  336*pow(t,5),  504*pow(t,6),
    0., 0.,        0.,          0.,          24.,       120*t, 360*pow(t,2), 840*pow(t,3), 1680*pow(t,4), 3024*pow(t,5);
    
    return table;
}
MatrixXd getCostfn_poly(double t)
{
    MatrixXd costfn_poly(10,10);

    costfn_poly<<
    0., 0., 0., 0.,              0.,              0.,               0.,               0.,                0.,                    0,
    0., 0., 0., 0.,              0.,              0.,               0.,               0.,                0.,                    0,
    0., 0., 0., 0.,              0.,              0.,               0.,               0.,                0.,                    0,
    0., 0., 0., 0.,              0.,              0.,               0.,               0.,                0.,                    0,
    0., 0., 0., 0.,          576*t,  1440*pow(t,2),   2880*pow(t,3),   5040*pow(t,4),    8064*pow(t,5),       12096*pow(t,6),
    0., 0., 0., 0.,  1440*pow(t,2),  4800*pow(t,3),  10800*pow(t,4),  20160*pow(t,5),   33600*pow(t,6),       51840*pow(t,7),
    0., 0., 0., 0.,  2880*pow(t,3), 10800*pow(t,4),  25920*pow(t,5),  50400*pow(t,6),   86400*pow(t,7),      136080*pow(t,8),
    0., 0., 0., 0.,  5040*pow(t,4), 20160*pow(t,5),  50400*pow(t,6), 100800*pow(t,7),  176400*pow(t,8),      282240*pow(t,9),
    0., 0., 0., 0.,  8064*pow(t,5), 33600*pow(t,6),  86400*pow(t,7), 176400*pow(t,8),  313600*pow(t,9),     508032*pow(t,10),
    0., 0., 0., 0., 12096*pow(t,6), 51840*pow(t,7), 136080*pow(t,8), 282240*pow(t,9), 508032*pow(t,10), 9144576*pow(t,11)/11.0;
    return costfn_poly;
}
MatrixXd getVAPoly(double t)
{
    MatrixXd table(2,10);
    table <<
    0., 1.,      2*t, 3*pow(t,2),  4*pow(t,3),  5*pow(t,4),   6*pow(t,5),   7*pow(t,6),    8*pow(t,7),    9*pow(t,8),
    0., 0.,        2.,        6*t, 12*pow(t,2), 20*pow(t,3),  30*pow(t,4),  42*pow(t,5),   56*pow(t,6),   72*pow(t,7);
    return table;
}
MatrixXd blkdiag(MatrixXd a,MatrixXd b){
    MatrixXd c;
    if (a.size() == 0) {
        c = b;
    }
    else if (b.size() == 0){
        c = a;
    }
    else{
        MatrixXd zero1 = MatrixXd::Constant(a.rows(),b.cols(),0.);
        MatrixXd zero2 = MatrixXd::Constant(b.rows(),a.cols(),0.);
        MatrixXd table(a.rows()+b.rows(),a.cols()+b.cols());
        table<<a,zero1,
        zero2,b;
        c = table;
    }
    return c;
}

MatrixXd QP(MatrixXd keyframe,MatrixXd time_kf,double v1 ,double a1)
{
  //std::cout << keyframe << std::endl;
  //std::cout << time_kf << std::endl;
    int deg = 9;
    int reldeg = 4;
    int num_kf = (int)keyframe.cols();
    
    MatrixXd H;
    
    for (int i = 0; i < num_kf-1; i++) {
        MatrixXd tem = getCostfn_poly(time_kf(0,i+1))-getCostfn_poly(time_kf(0,i));
        H = blkdiag(H, tem);
    }
    
    MatrixXd A;
    for (int i = 0; i < num_kf-1; i++) {
        MatrixXd tem = getAllTable(time_kf(0,i));
        MatrixXd tem2 = getAllTable(time_kf(0,i+1));
        MatrixXd x(2*tem.rows(),1*tem.cols());
        x <<
        tem,
        tem2;
        A = blkdiag(A,x);
    }
    double totaldeg=1+reldeg;
    double dgbegin = 3;
    double deend = 3;
    
    MatrixXd fixdg = MatrixXd::Constant(1,num_kf,1);
    fixdg(0,0) = dgbegin;
    fixdg(0,num_kf-1) = deend;
    MatrixXd freedg = totaldeg*MatrixXd::Constant(1,num_kf,1)-fixdg;
    MatrixXd M = MatrixXd::Constant(A.cols(),totaldeg*num_kf,0);
    for (int i = 0; i < fixdg(0,0); i++) {
        M(i,i)=1;
        
    }
    for (int i = 0; i < freedg(0,0); i++) {
        M(i+fixdg(0,0),fixdg.sum()+i)=1;
        
    }
    for (int j = 2; j <= num_kf ; j++) {
        for (int i = 1; i <= fixdg(0,j-1); i++) {
            M((2*j-3)*totaldeg+i-1,fixdg.block(0, 0, 1, j-1).sum()+i-1)=1;
        }
        for (int i = 1; i <= freedg(0,j-1); i++) {
            M((2*j-3)*totaldeg+fixdg(0,j-1)+i-1,fixdg.sum()+freedg.block(0, 0, 1, j-1).sum()+i-1)=1;
        }
        if (j != num_kf) {
            for (int i = 1; i <= totaldeg; i++) {
                M.block((2*j-2)*totaldeg+i-1, 0, totaldeg-i+1, M.cols())
                = M.block((2*j-3)*totaldeg+i-1,0,totaldeg-i+1,M.cols()).eval();
            }
        }
    }
    
    M.transposeInPlace();
    
    MatrixXd Df(1,num_kf+4);
    Df << keyframe.block(0, 0, 1, 1), v1, a1,keyframe.block(0, 1, 1, num_kf-1), 0, 0;
    Df.transposeInPlace();
    
    MatrixXd invA = A.inverse();
    
    MatrixXd R = M*invA.transpose()*H*invA*M.transpose();
    
    MatrixXd Rfp = R.block(0, fixdg.sum()+1-1, fixdg.sum(), R.cols()-fixdg.sum());
    
    MatrixXd Rpp = R.block(fixdg.sum()+1-1, fixdg.sum()+1-1, R.rows()-fixdg.sum(), R.cols()-fixdg.sum());
    
    MatrixXd Dp = Rpp.inverse()*Rfp.transpose()*Df;
    Dp = Dp*-1;
    
    
    MatrixXd tem(Df.rows()+Dp.rows(),Dp.cols());
    tem <<
    Df,
    Dp;
    
    MatrixXd Draw = M.transpose()*tem;
    MatrixXd uncons_coef = A.inverse()*Draw;

    MatrixXd uncons_coef2(deg+1,num_kf-1);
    for (int i = 0; i < num_kf -1; i++) {
        uncons_coef2.col(i) = uncons_coef.block(i*(deg+1), 0, deg+1, 1);
    }
    //std::cout << uncons_coef2 << std::endl;

    return uncons_coef2;
}
std::vector< double > getTraj(MatrixXd optim_poly,MatrixXd time_kf){
    std::vector< double >traj;
    
    for (int i = 0; i < optim_poly.cols() ; i++) {
        for (double j = time_kf(0,i); j < time_kf(0,i+1); j=j+0.005) {
            MatrixXd x =getPoly(j)*(optim_poly.col(i));
            traj.push_back(x(0,0));
            //std::cout << j << " and " << x(0,0) << std::endl;
        }
    }
    return traj;
}

std::vector< double > getVandA(MatrixXd optim_poly,MatrixXd time_kf,double t){
    std::vector< double >traj;
    
    for (int i = 0; i < optim_poly.cols(); i++) {
        if ( t < time_kf(0,i)) {
            MatrixXd x =getVAPoly(t)*(optim_poly.col(i));
            traj.push_back(x(0,0));
            traj.push_back(x(1,0));
           // std::cout << t << " and " << x(0,0)<< x(1,0) << std::endl;
            break;
        }
    }
    
    return traj;
}









