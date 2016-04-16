#ifndef GEO_DATATYPE_H
#define GEO_DATATYPE_H

#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>

//typedef CGAL::Cartesian<double> Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel    Kernel;
typedef Kernel::Point_2                                      Point2D;
typedef CGAL::Polygon_2<Kernel>                              Polygon2D;

#endif // GEO_DATATYPE_H
