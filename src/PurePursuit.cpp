#include <ras_group8_navigation/PurePursuit.hpp>
#include <geometry_msgs/Pose.h>

// STD
#include <string.h>
#include <math.h>

namespace ras_group8_navigation {

static inline void
  addPoints(const Point *A, const Point *B, Point *C);
  
static inline void
  subtractPoints(const Point *A, const Point *B, Point *AB);
  
static inline double
  scalarProductOfPoints(const Point *A, const Point *B);
  
static inline void
  multiplyPoint(const Point *A, double scalar, Point *A_scaled);
  
static inline double
  squarePoint(const Point *A);
  
static inline double
  distanceBetweenPoints(const Point *A, const Point *B);
  
static inline void
  projectPoint(const Point *A, const Point *AB, double t, Point *p);
 

PurePursuit::PurePursuit()
  : lookahead_(0), lookahead_squared_(0)
{
}

PurePursuit::PurePursuit(const nav_msgs::Path& path_msg, double lookahead)
  : lookahead_(lookahead), lookahead_squared_(lookahead * lookahead)
{
  const int segments_len = path_msg.poses.size() - 1;
  
  /* We assume there are at least one line segment in the path */
  ROS_ASSERT(segments_len >= 1);
  
  segments_.resize(segments_len);
  
  Point J;
  Point *A;
  Point *B = &J;
  
  J.x = path_msg.poses[0].pose.position.x;
  J.y = path_msg.poses[0].pose.position.y;
  
  /* Pre calculate what we can here */
  for (int i = 0; i < segments_len; i ++) {
    const geometry_msgs::Pose *pose = &path_msg.poses[i + 1].pose;
    Segment *segment = &segments_[i];
    
    A = &segment->A;
    
    A->x = B->x;
    A->y = B->y;
      
    B->x = pose->position.x;
    B->y = pose->position.y;
    
    subtractPoints(A, B, &segment->AB);
    segment->AB_squared = squarePoint(&segment->AB);
  }
}

PurePursuit::~PurePursuit()
{
}

geometry_msgs::Pose
PurePursuit::nextPose(const geometry_msgs::Pose& current_pose)
{
  Point o;
  const int segments_len = segments_.size();
  
  o.x = current_pose.position.x;
  o.y = current_pose.position.y;
  
  /* Calculate the distance from the current pose to each
     line segment */
  for (int i = 0; i < segments_len; i++) {
    Segment *segment = &segments_[i];
    Point Ap;
    Point p;
    subtractPoints(&segment->A, &o, &Ap);
    
    segment->t = scalarProductOfPoints(&Ap, &segment->AB)
                   / segment->AB_squared;
                       
    if (segment->t < 0.0) {
      p.x = segment->A.x;
      p.y = segment->A.y;
    } else if (segment->t > 1.0) {
      addPoints(&segment->A, &segment->AB, &p);
    } else {
      projectPoint(&segment->A, &segment->AB, segment->t, &p);
    }
    
    segment->d = distanceBetweenPoints(&o, &p);
  }
  
  /* Find the smallest distance */
  int d_min_i  = 0;
  double d_min = segments_[0].d;
  for (int i = 1; i < segments_len; i++) {
    const double d = segments_[i].d;
    if (d <= d_min) { /* We want the last closest */
      d_min   = d;
      d_min_i = i;
    }
  }
  
  Point q;
  /* Check if d is smaller than the lookahead */
  if (d_min < lookahead_) {
    while (d_min_i < segments_len) {
      Point p;
      Segment *segment = &segments_[d_min_i];
      
      projectPoint(&segment->A, &segment->AB, segment->t, &p);
      
      const double d = distanceBetweenPoints(&o, &p);
      /* Project forward onto the line */
      const double tp = segment->t +
        sqrtf((lookahead_squared_ - d*d) / segment->AB_squared);
        
      if (tp > 1.0) {
        if (d_min_i == segments_len - 1) {
          addPoints(&segment->A, &segment->AB, &q);
          break;
        } else {
          d_min_i ++;
        }
      } else {
        projectPoint(&segment->A, &segment->AB, tp, &q);
        break;
      }
    }
  } else {
    const Segment *segment = &segments_[d_min_i];
    projectPoint(&segment->A, &segment->AB, segment->t, &q);
  }
  
  geometry_msgs::Pose next_pose;
  
  next_pose.position.x = q.x;
  next_pose.position.y = q.y;
  
  return next_pose;
}

void
addPoints(const Point *A, const Point *B, Point *C)
{
  C->x = B->x + A->x;
  C->y = B->y + A->y;
}

void
subtractPoints(const Point *A, const Point *B, Point *AB)
{
  AB->x = B->x - A->x;
  AB->y = B->y - A->y;
}

double
scalarProductOfPoints(const Point *A, const Point *B)
{
  return (A->x * B->x) + (A->y * B->y);
}

void
multiplyPoint(const Point *A, double scalar, Point *A_scaled)
{
  A_scaled->x = A->x * scalar;
  A_scaled->y = A->y * scalar;
}

double
squarePoint(const Point *A)
{
  return scalarProductOfPoints(A, A);
}

double
distanceBetweenPoints(const Point *A, const Point *B)
{
  const double dx = A->x - B->x;
  const double dy = A->y - B->y;
  
  return sqrtf(dx*dx + dy*dy);
}

void
projectPoint(const Point *A, const Point *AB, double t, Point *p)
{
  multiplyPoint(AB, t, p);
  addPoints(A, p, p);
}

}