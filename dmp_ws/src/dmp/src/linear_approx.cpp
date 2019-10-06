/*********************************************************************
DMP created in 2019.09
 *********************************************************************/

#include "dmp/linear_approx.h"
#include<stdio.h>
using namespace std;

namespace dmp{
    
    
bool sort_pt_pair(const pt_pair& left, const pt_pair& right)
{
    return left.first < right.first;
}    


LinearApprox::LinearApprox()
{
	n_bases = 0;
}


LinearApprox::LinearApprox(std::vector<double> X, std::vector<double> Y)
{
        n_bases = X.size();
        
        // Read in points and sort them by X-value
        for(int i=0; i<n_bases; i++){
            points.push_back(pt_pair(X[i], Y[i]));
        }
        
        std::sort(points.begin(), points.end(), sort_pt_pair);
        
}


LinearApprox::~LinearApprox(){};


//Assumes that x is between 0 and 1 inclusive and that fxn is zero at x=0 and x=1 if not specified
//Perform linear interpolation between saved points
double LinearApprox::evalAt(double x)
{
        // If out of bounds, or x=0, return 0
        if(x <= 0.0 || x > 1.0) 
            return 0.0;
    
        //If not points to interp, return 0
        if(n_bases <= 0)
            return 0.0;
                
        // If less than the smallest entry, interp with x=0
        if(x < points[0].first){
            double slope = points[0].second / points[0].first;
            return slope * x;
        }    
        // If greater than largest entry, interp with fxn=0 at x=1
        else if(x > points[n_bases-1].first){    
            double inv_slope = points[n_bases].second / points[n_bases].first; 
            return inv_slope * (1.0 - x);
        }
        // Otherwise, normal interp
        else{
            double curr = 0.0;
            int i = 0;
        
            while(x > curr && i < n_bases){
                i++;
                curr = points[i].first;
            }
            
            double diffx = points[i].first - points[i-1].first;
            double diffy = points[i].second - points[i-1].second;
            double slope = diffy/diffx;
                
            double y_start =  points[i-1].second;
            double x_dist = x - points[i-1].first;
            return y_start + (x_dist * slope);
            
        }
            
            
}

// Nothing to do here, weights are not used
void LinearApprox::leastSquaresWeights(double *X, double *Y, int n_pts){};

}


