#include <cmath>
#include <iostream>
#include <cstring>
#include <limits>
#include <algorithm>

#include "hungarian_solver.h"

void CHungarianSolver::Reset() {
   max_match = 0; //number of vertices in current matching
   memset(xy, -1, sizeof(xy)); 
   memset(yx, -1, sizeof(yx));
   //std::fill(std::begin(xy), std::end(xy), -1);
   //std::fill(std::begin(yx), std::end(yx), -1);
}

double CHungarianSolver::operator()(const TCostMatrix& t_cost_matrix) {
   n = t_cost_matrix.size();

   Reset();
   InitLabels(t_cost_matrix);
   Augment(t_cost_matrix);

   double ret = 0.0f;
   for (int x = 0; x < n; x++) //forming answer there
      ret += t_cost_matrix[x][xy[x]];
   return ret;
}

void CHungarianSolver::InitLabels(const TCostMatrix& t_cost_matrix)
{
  memset(lx, 0, sizeof(lx));
  memset(ly, 0, sizeof(ly));
  for (int x = 0; x < n; x++)
    for (int y = 0; y < n; y++)
      lx[x] = std::max(lx[x], t_cost_matrix[x][y]);
}

void CHungarianSolver::UpdateLabels()
{
  int x, y;
  double delta = std::numeric_limits<double>::infinity(); //init delta as infinity
  for (y = 0; y < n; y++) //calculate delta using slack
    if (!T[y])
      delta = std::min(delta, slack[y]);
  for (x = 0; x < n; x++) //update X labels
    if (S[x]) lx[x] -= delta;
  for (y = 0; y < n; y++) //update Y labels
    if (T[y]) ly[y] += delta;
  for (y = 0; y < n; y++) //update slack array
    if (!T[y])
      slack[y] -= delta;
}

//x - current vertex,prevx - vertex from X before x in the alternating path,
//so we add edges (prevx, xy[x]), (xy[x], x)
void CHungarianSolver::AddToTree(const TCostMatrix& t_cost_matrix, int x, int prevx) {
  S[x] = true; //add x to S
  prev[x] = prevx; //we need this when augmenting
  for (int y = 0; y < n; y++) //update slacks, because we add new vertex to S
    if (lx[x] + ly[y] - t_cost_matrix[x][y] < slack[y]) {
        slack[y] = lx[x] + ly[y] - t_cost_matrix[x][y];
        slackx[y] = x;
      }
}

void CHungarianSolver::Augment(const TCostMatrix& t_cost_matrix) //main function of the algorithm
{
  if (max_match == n) return; //check wether matching is already perfect
  int x, y, root; //just counters and root vertex
  int q[N], wr = 0, rd = 0; //q - queue for bfs, wr,rd - write and read
  //pos in queue
  memset(S, false, sizeof(S)); //init set S
  memset(T, false, sizeof(T)); //init set T
  memset(prev, -1, sizeof(prev)); //init set prev - for the alternating tree
  for (x = 0; x < n; x++) { //finding root of the tree
    if (xy[x] == -1) {
      q[wr++] = root = x;
      prev[x] = -2;
      S[x] = true;
      break;
    }
  }
  for (y = 0; y < n; y++) { //initializing slack array

      slack[y] = lx[root] + ly[y] - t_cost_matrix[root][y];
      slackx[y] = root;
    }
  //second part of augment() function
  for(;;) { //main cycle
      while (rd < wr) { //building tree with bfs cycle

          x = q[rd++]; //current vertex from X part
          for (y = 0; y < n; y++) //iterate through all edges in equality graph
            if (t_cost_matrix[x][y] == lx[x] + ly[y] && !T[y]) {
                if (yx[y] == -1) break; //an exposed vertex in Y found, so
                //augmenting path exists!
                T[y] = true; //else just add y to T,
                q[wr++] = yx[y]; //add vertex yx[y], which is matched
                //with y, to the queue
                AddToTree(t_cost_matrix, yx[y], x); //add edges (x,y) and (y,yx[y]) to the tree
              }
          if (y < n) break; //augmenting path found!
        }
      if (y < n) break; //augmenting path found!

      UpdateLabels(); //augmenting path not found, so improve labeling
      wr = rd = 0; 
      for (y = 0; y < n; y++) 
        //in this cycle we add edges that were added to the equality graph as a
        //result of improving the labeling, we add edge (slackx[y], y) to the tree if
        //and only if !T[y] && slack[y] == 0, also with this edge we add another one
        //(y, yx[y]) or augment the matching, if y was exposed
        if (!T[y] && slack[y] == 0) {
            if (yx[y] == -1) { //exposed vertex in Y found - augmenting path exists!
                x = slackx[y];
                break;
            }
            else {
                T[y] = true; //else just add y to T,
                if (!S[yx[y]]) {
                    q[wr++] = yx[y]; //add vertex yx[y], which is matched with
                    //y, to the queue
                    AddToTree(t_cost_matrix, yx[y], slackx[y]); //and add edges (x,y) and (y,
                    //yx[y]) to the tree
                  }
              }
          }
      if (y < n) break; //augmenting path found!
    }

  if (y < n) { //we found augmenting path!
         max_match++; //increment matching
      //in this cycle we inverse edges along augmenting path
      for (int cx = x, cy = y, ty; cx != -2; cx = prev[cx], cy = ty) {
          ty = xy[cx];
          yx[cy] = cx;
          xy[cx] = cy;
      }
      Augment(t_cost_matrix); //recall function, go to step 1 of the algorithm
    }
}//end of augment() function

std::vector<std::pair<int, int> >  CHungarianSolver::GetAssignments() {
   std::vector<std::pair<int, int> >  vecToReturn;
   for (int x = 0; x < n; x++)
      vecToReturn.emplace_back(x, xy[x]);
   return vecToReturn;
}




