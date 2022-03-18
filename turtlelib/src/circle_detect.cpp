#include <iostream>
#include "turtlelib/circle_detect.hpp"
#include <string>
#include <cmath>
#include <armadillo>

namespace turtlelib{
    Circle::Circle(){}

    Circle::Circle(std::vector<std::vector<Vector2D>> xy_clusters):xy_clusters(xy_clusters){}

    void Circle::range_cluster(std::vector<float> ranges,float thresh_dist){
        std::vector<float> cur_cluster;
        std::vector<Vector2D> cur_cluster_xy;
        double angle_increment = 0.0174;
        cur_cluster_xy.push_back({ranges[0]*cos(0),ranges[0]*sin(0)});
        cur_cluster.push_back(0);
        for(int i=1;i<ranges.size();i++){
            if(abs(ranges[i]-ranges[i-1])<=thresh_dist){
                cur_cluster.push_back(i);
                cur_cluster_xy.push_back({ranges[i]*cos(i*angle_increment),ranges[i]*sin(i*angle_increment)});
            }
            else{
                //only take clusters larger than 3 points
                if(cur_cluster.size()>3) {
                    clusters.push_back(cur_cluster);
                    xy_clusters.push_back(cur_cluster_xy);
                }
                //start new cluster search
                cur_cluster.clear();
                cur_cluster_xy.clear();
                cur_cluster.push_back(i);
                cur_cluster_xy.push_back({ranges[i]*cos(i*angle_increment),ranges[i]*sin(i*angle_increment)});
            }
        }
        clusters.push_back(cur_cluster);
        xy_clusters.push_back(cur_cluster_xy);
        //check if the start and end belongs to same cluster
        if ((cur_cluster[cur_cluster.size()-1]==(ranges.size()-1))&&(clusters[0][0]==1)
                &&(abs(ranges[0]-ranges[ranges.size()-1])<=thresh_dist))
        {   
            //insert the last cluster to the beginning of the first cluster
            clusters[0].insert(std::begin(clusters[0]), std::begin(cur_cluster), std::end(cur_cluster));
            xy_clusters[0].insert(std::begin(xy_clusters[0]), std::begin(cur_cluster_xy), std::end(cur_cluster_xy));
            clusters.pop_back();
            xy_clusters.pop_back();
        }
    }
    void Circle::circle_fitting(){
        for(int i=0;i<xy_clusters.size();i++){
            Vector2D xy_sum = {0,0};
            int n = xy_clusters[i].size();
            for(int j=0;j<n;j++){
                xy_sum += xy_clusters[i][j];
            }
            Vector2D xy_mean = xy_sum*(1.0/n);
            arma::mat Z = arma::zeros<arma::mat>(n,4);
            double z_mean = 0;
            for(int j=0;j<n;j++){
                Vector2D xy = xy_clusters[i][j]-xy_mean; 
                Z(j,0) = xy.x*xy.x+xy.y*xy.y;
                Z(j,1) = xy.x;
                Z(j,2) = xy.y;
                Z(j,3) = 1;
            }
            arma::rowvec row_sum = arma::sum(Z,0);
            z_mean = row_sum(0)/n;
            arma::mat M = (Z.t()*Z)/n; 
            arma::mat H_inv = arma::zeros<arma::mat>(4,4);
            H_inv(0,3) = 0.5;
            H_inv(1,1) = 1;
            H_inv(2,2) = 1;
            H_inv(3,0) = 0.5;
            H_inv(3,3) = -2*z_mean;

            arma::mat U;
            arma::mat V;
            arma::vec sigma;
            arma::svd(U,sigma,V,Z);
            arma::vec A;
            double min_sigma = arma::min(sigma);
            if (min_sigma<pow(10,-12)) {
                std::cout <<"correct" << std::endl;
                std::cout << "V" << V <<std::endl; 
                A = V.col(3);
            }
            else{
                arma::mat bigsigma = arma::eye<arma::mat>(4,4);
                bigsigma(0,0) = sigma(0);
                bigsigma(1,1) = sigma(1);
                bigsigma(2,2) = sigma(2);
                bigsigma(3,3) = sigma(3);
                arma::mat Y = V*bigsigma*V.t();
                arma::mat Q = Y*H_inv*Y;
                arma::cx_vec eigval;
                arma::cx_mat eigvec;
                arma::eig_gen(eigval,eigvec,Q);
                double min_eig = (eigval[0].real())*(eigval[0].real())+(eigval[0].imag())*(eigval[0].imag());
                int min_idx = 0;
                for(int i=1;i<eigval.size();i++){
                    double cur_eig = (eigval[i].real())*(eigval[i].real())+(eigval[i].imag())*(eigval[i].imag());
                    if (cur_eig<min_eig){
                        min_idx = i;
                        min_eig = cur_eig;
                    }
                }
                arma::cx_vec A_star = eigvec.col(min_idx);

                A = inv(Y)*real(A_star);
            }
            //solve for the equation for the circle
            double a = -A(1)/(2*A(0));
            double b = -A(2)/(2*A(0));
            double R = sqrt((A(1)*A(1)+A(2)*A(2)-4*A(0)*A(3))/(4*A(0)*A(0)));
            if (R>0.1)continue;
            Vector2D center = {xy_mean.x+a,xy_mean.y+b};    
            centers.push_back(center);        
            radius.push_back(R);
        }
    }
    void Circle::classification(){
        std::vector<std::vector<Vector2D>> new_xy_clusters;
        for(int i=0;i<xy_clusters.size();i++){
            std::vector<double>angles;
            double sum_angle = 0;
            for(int j=1;j<xy_clusters[i].size()-1;j++){
                //compute the angle between lines (P_random,P_first) (P_random,Plast)
                double p_angle = angle(xy_clusters[i][j]-xy_clusters[i][0],\
                            xy_clusters[i][j]-xy_clusters[i][xy_clusters[i].size()-1]);
                angles.push_back(p_angle);
                sum_angle+=p_angle;
            }
            double mean_angle = sum_angle/(xy_clusters[i].size()-2);
            double std_angle=0;
            //compute mean and standard derivation of the angles
            for(int j=0;j<angles.size();j++){
                std_angle+=(angles[j]-mean_angle)*(angles[j]-mean_angle);
            }
            std_angle = sqrt(std_angle/(xy_clusters[i].size()-2));
            if(std_angle<2 && rad2deg(mean_angle)>90 && rad2deg(mean_angle)<140)
                new_xy_clusters.push_back(xy_clusters[i]);
            
        }
        xy_clusters = new_xy_clusters;
    }

    void Circle::clear(){
        clusters.clear();
        xy_clusters.clear();
        centers.clear();
        radius.clear();
    }


}