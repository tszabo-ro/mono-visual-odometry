#ifndef Primitives_h
#define Primitives_h

template<class T>
struct Rect
{
  Rect(T start_x, T start_y, T end_x, T end_y)
  : start_x(start_x), start_y(start_y), end_x(end_x), end_y(end_y)
  {}

  T width() const { return end_x - start_x; }
  T height() const { return end_y - start_y; }

  T start_x;
  T start_y;
  T end_x;
  T end_y;
};

template<class T>
struct Point2
{
  Point2(T x, T y)
  : x(x), y(y) {}

  const T x;
  const T y;
};

using Point2i = Point2<int>;
using Point2f = Point2<float>;
using Point2d = Point2<double>;

template<class T>
struct Vector2
{
  Vector2(T x, T y)
  : x(x), y(y) {}

  const T x;
  const T y;
};

using Vector2i = Vector2<int>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;

template<class T, class Q>
Point2<T> operator+(Point2<T> pt, Vector2<Q> vec) { return Point2<T>(pt.x + vec.x, pt.y + vec.y); }

template<class T, class Q>
Vector2<T> operator+(Vector2<T> pt, Vector2<Q> vec) { return Vector2<T>(pt.x + vec.x, pt.y + vec.y); }


#endif
