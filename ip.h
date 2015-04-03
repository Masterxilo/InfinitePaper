struct Point {
  double x, y;
  Point(double x, double y) : x(x), y(y) {}
};

bool operator==(Point& a, Point& b) {
  return a.x == b.x && a.y == b.y;
}
struct PointOrNull {
  double x, y;
  bool isNull;
  PointOrNull(double x, double y) : x(x), y(y), isNull(false) {}
  PointOrNull() : isNull(true) {}
};

bool operator==(PointOrNull& a, PointOrNull& b) {
  return a.isNull && b.isNull ||
    !a.isNull && !b.isNull && a.x == b.x && a.y == b.y
    ;
}

bool operator!=(PointOrNull& a, PointOrNull& b) {
  return !(a == b);
}

struct TwoPointOrNulls {
  PointOrNull p1, p2;
  TwoPointOrNulls(PointOrNull p1, PointOrNull p2) : p1(p1), p2(p2) {}
  TwoPointOrNulls() {}
};

bool operator==(TwoPointOrNulls& a, TwoPointOrNulls& b) {
  return a.p1 == b.p1 && a.p2 == b.p2
    ;
}


struct NullOrTwoPoints {
  Point p1, p2;
  bool isNull;
  NullOrTwoPoints(Point p1, Point p2) : p1(p1), p2(p2), isNull(false) {}
  NullOrTwoPoints() : isNull(true), p1(Point(0, 0)), p2(Point(0, 0)) {}
};

bool operator==(NullOrTwoPoints& a, NullOrTwoPoints& b) {
  return a.isNull && b.isNull ||
    !a.isNull && !b.isNull && a.p1 == b.p1 && a.p2 == b.p2
    ;
}

#define _USE_MATH_DEFINES
#include <math.h>
struct R2toR2Rigid {
  double scale, theta;
  Point move1, move2;

  R2toR2Rigid() :
    move1(Point(0, 0)), scale(1), theta(0), move2(Point(0, 0)) {}


  R2toR2Rigid(Point move1, double scale, double theta, Point move2) :
    move1(move1), scale(scale), theta(theta), move2(move2) {}

  Point operator() (Point p) {
    Point o = p;
    o.x += move1.x;
    o.y += move1.y;

    // change of basis to
    /*
    cos(theta) -sin(theta)
    sin(theta) cos(theta)
    */
    Point tt = o;
    o.x = tt.x * cos(theta) - tt.y * sin(theta);
    o.y = tt.x * sin(theta) + tt.y * cos(theta);

    o.x *= scale;
    o.y *= scale;

    o.x += move2.x;
    o.y += move2.y;
    return o;
  }
};

double angle(Point p) {
  return atan2(p.y, p.x);
}

double length(Point p) {
  return sqrt(p.x*p.x + p.y*p.y);
}

R2toR2Rigid rt(Point a, Point b, Point c, Point d) {
  Point move1 = a;
  move1.x *= -1; move1.y *= -1;

  Point d1 = b;
  d1.x -= a.x; d1.y -= a.y;
  Point d2 = d;
  d2.x -= c.x; d2.y -= c.y;

  double angle1 = angle(d1);
  double angle2 = angle(d2);
  double angle = angle2 - angle1;

  return R2toR2Rigid(move1, length(d2) / length(d1), angle, c);
}

const double eps = 0.000001;
bool approx(double a, double b) {
  return abs(b - a) < eps;
}
bool approx(Point x, Point y) {
  return approx(x.x, y.x) && approx(x.y, y.y);
}


#include <assert.h>

NullOrTwoPoints computeTb(TwoPointOrNulls t, TwoPointOrNulls t_) {
  if (t.p1.isNull && t.p2.isNull) return NullOrTwoPoints();


  // Discard sudden change of finger or change between 
  // move <-> and scale/rotate modes to avoid jumps
  /*
  t(i-1)      t(i)    tb(i)
  E*          *E      EE
  E*          **      EE
  *E          E*      EE
  *E          **      EE
  */
#define EEcase(tr) \
  if (((t_.p1 == PointOrNull() && tr[0] == 'E') || (t_.p1 != PointOrNull() && tr[0] == '*')) && \
  ((t_.p2 == PointOrNull() && tr[1] == 'E') || (t_.p2 != PointOrNull() && tr[1] == '*')) && \
  ((t.p1 == PointOrNull() && tr[3] == 'E') || (t.p1 != PointOrNull() && tr[3] == '*')) && \
  ((t.p2 == PointOrNull() && tr[4] == 'E') || (t.p2 != PointOrNull() && tr[4] == '*'))) return NullOrTwoPoints();


  EEcase("E* *E")
    EEcase("E* **")
    EEcase("*E E*")
    EEcase("*E **")
    EEcase("** *E")
    EEcase("** E*")

    // Forward correct input
  if (!t.p1.isNull && !t.p2.isNull) {
    Point p1 = Point(t.p1.x, t.p1.y);
    Point p2 = Point(t.p2.x, t.p2.y);
    return NullOrTwoPoints(p1, p2);
  }

  // Emulate second finger for movement
  if (t.p1.isNull && !t.p2.isNull) {
    Point p2 = Point(t.p2.x, t.p2.y);
    Point p1 = p2; p1.x += 1; // slight change, arbitrary
    return NullOrTwoPoints(p1, p2);
  }
  if (!t.p1.isNull && t.p2.isNull) {
    Point p1 = Point(t.p1.x, t.p1.y);
    Point p2 = p1; p2.x += 1; // slight change, arbitrary
    return NullOrTwoPoints(p1, p2);
  }
  assert(false);
  return NullOrTwoPoints();
}

#include <stdio.h>

// Program
// _ means at the last time
TwoPointOrNulls t, t_;
NullOrTwoPoints tb, tb_;
R2toR2Rigid f, fi, f_, fi_;

int i = 0;
void step() {
  i++;
  // Update tb
  tb = computeTb(t, t_);

  // Update f
  if (tb == NullOrTwoPoints() || tb_ == NullOrTwoPoints()) {
    // Nothing changes if we don't have touchpoints for this and the last step
    f = f_;
    fi = fi_;
  }
  else {
    // Change f based on tb and tb_
    // World coordinates of previous touch points
    Point A = fi_(tb_.p1);
    Point B = fi_(tb_.p2);
    //
    f = rt(
      A, B,
      tb.p1, tb.p2 // Must map to new screen coordinates
      );
    // inverse
    fi = rt(
      tb.p1, tb.p2,
      A, B
      );
  }


  // do something
  printf("==== i %d ====\n", i);
  // t
  if (t.p1.isNull)
  if (t.p2.isNull)
    printf("t null null\n");
  else
    printf("t null (%f %f)\n", t.p2.x, t.p2.y);
  else
  if (t.p2.isNull)
    printf("t (%f %f) null\n", t.p1.x, t.p1.y);
  else
    printf("t (%f %f) (%f %f)\n", t.p1.x, t.p1.y, t.p2.x, t.p2.y);

  // tb
  if (tb.isNull)
    printf("tb null\n");
  else
    printf("tb (%f %f) (%f %f)\n", tb.p1.x, tb.p1.y, tb.p2.x, tb.p2.y);

  // test f
  Point tf = f(Point(0, 0));
  printf("world 0, 0 -> screen [f(0, 0)] == %f %f\n", tf.x, tf.y);

  tf = f(Point(1, 0));
  printf("world 1, 0 -> screen [f(1, 0)] == %f %f\n", tf.x, tf.y);

  tf = f(Point(0, 1));
  printf("world 0, 1 -> screen [f(0, 1)] == %f %f\n", tf.x, tf.y);

  tf = f(Point(1, 1));
  printf("world 1, 1 -> screen [f(1, 1)] == %f %f\n", tf.x, tf.y);


  // Update last value
  t_ = t;
  tb_ = tb;
  f_ = f;
  fi_ = fi;
}