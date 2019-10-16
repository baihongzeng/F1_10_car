#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

      c.clear();
      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;

      //Do for each point, find the corresponding old_points of the transformed cur_points
      for(int i = 0; i<n; ++i){
        min_dist = 100000.00;
        min_index = 0;
        second_min_index = 0;
        for(int j = 0; j<m; ++j){
          //float dist = old_points[i].distToPoint2(&trans_points[j]);
          float dist = old_points[j].distToPoint2(&trans_points[i]);
          if(dist<min_dist){
            min_dist = dist;
            min_index = j;
            //second_min_index = j-1;

          if(min_index <= 0){
              second_min_index = min_index + 1;
          }
          else{second_min_index = min_index - 1;}
          }
        }
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }


}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a reference which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point. 
  //Initialize correspondences
  c.clear();
  //ROS_INFO("1");
  int last_best = -1;//last_best = invalid
  const int n = trans_points.size();
  const int m = old_points.size();
  const int nrays = trans_points.size();
  //Do for each point
  for(int i = 0; i<n; ++i){

    
    // Current best match, and its distance
    int best = 0;
    int second_best = 0;//second best 先不用管，找到best 就有second best

    ////自己写的部分
    double best_dist = DBL_MAX;

    //Approximated index in previous scan corresponding to point: trans_points
    int start_index = (trans_points[i].theta - old_points[0].theta) * nrays / (2 * M_PI);
    //restricted the range of start_index (belong to old_points category)
    if(start_index <= 0){start_index = 0;}
    else if(start_index >= m){start_index = m - 1;}

    //If last match was successful, then start at that index + 1
    int we_start_at = (last_best != -1) ? (last_best + 1) : start_index; //若不是在初始状态，从上一个点对应的best + 1的index开始，否则从这个点自身的index开始
    //Search is conducted in two directions: UP and DOWN
    int up = we_start_at + 1;
    int down = we_start_at;
    //Distance of last point examined in the up (down) direction
    double last_dist_up = 100000 - 1;
    double last_dist_down = 100000;
    //True if search is finished in the up (down) direction
    bool up_stopped = false;
    bool down_stopped = false;

    //Until the search is stopped in both directions...
    while(up_stopped == false || down_stopped == false){ //we still have one direction that isn't stopped yet
        //Should we try to explore up or down?
        bool now_up = !up_stopped && (last_dist_up < last_dist_down);//if up is not stopped yet AND last_dist_up is shorter
        if(now_up){
            //If we have finished the points to search, we stop
            if(up >= m){up_stopped = true; continue;}
            //Get the distance from current transformed point to the up point
            last_dist_up = trans_points[i].distToPoint2(&old_points[up]);
            //If it is less than the best point, up is our best guess so far
            if(last_dist_up < best_dist){
                best = up;
                best_dist = last_dist_up;
            }
            if(up > start_index){
                double delta_phi = (old_points[up].theta - trans_points[i].theta);
                double min_dist_up = sin(delta_phi) * trans_points[i].r;
                if(pow(min_dist_up, 2) > best_dist){
                    //If going up we can't make better than best_dist, then we stop searching in the "up" direction
                    up_stopped = true;
                    continue;
                }

                //If we are moving away, implement jump table optimization
                up = (old_points[up].r < trans_points[i].r) ? jump_table[up][UP_BIG] : jump_table[up][UP_SMALL];
            }
            else{//if we are moving towards "start cell", we can't do optimization, just move to the next point
                up++;
            }
        }//end of if(now_up)

        if(!now_up){//go down
            //If we have finished the points to search, we stop
            if(down <= -1){down_stopped = true; continue;}
            //Get the distance from current transformed point to the down point
            last_dist_down = trans_points[i].distToPoint2(&old_points[down]);
            //If it is less than the best point, 'down' is our best guess so far
            if(last_dist_down < best_dist){
                best = down;
                best_dist = last_dist_down;
            }
            if(down < start_index){
                double delta_phi = (trans_points[i].theta - old_points[down].theta);
                double min_dist_down = sin(delta_phi) * trans_points[i].r;
                if(pow(min_dist_down, 2) > best_dist){
                    //If going down we can't make better than best_dist, then we stop searching in the "down" direction
                    down_stopped = true;
                    continue;
                }

                //If we are moving away, implement jump table optimization
                down = (old_points[down].r < trans_points[i].r) ? jump_table[down][DOWN_BIG] : jump_table[down][DOWN_SMALL];
            }
            else{//if we are moving towards "start cell", we can't do optimization, just move to the next point
                down--;
            }
        }//end of if(!now_up)

    }
    /////缺少set null correspondence if no point matched
    
      //for the next point, we will start at 'best'
      last_best = best;

      if(best <= 0){
          second_best = best + 1;
      }
      else{second_best = best - 1;}

      c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
    }
  }


void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};//initialize every element of jump table
    //  vector<int> v = {n-1,n-1,0,0};//自己改的
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
