//! Integer polygon geometry for room outlines. Points on an edge count as inside.

/// Duty-count position
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point {
    pub x: u16,
    pub y: u16,
}

type V = (i64, i64);

fn v(p: Point) -> V {
    (p.x as i64, p.y as i64)
}

fn sub(a: V, b: V) -> V {
    (a.0 - b.0, a.1 - b.1)
}

fn cross(a: V, b: V) -> i64 {
    a.0 * b.1 - a.1 * b.0
}

fn dot(a: V, b: V) -> i64 {
    a.0 * b.0 + a.1 * b.1
}

/// Returns the sign of the turn from `a`-`b` to `a`-`c`
fn orient(a: V, b: V, c: V) -> i64 {
    cross(sub(b, a), sub(c, a)).signum()
}

/// Returns whether `p` lies on the closed segment `a`-`b`
fn on_segment(p: V, a: V, b: V) -> bool {
    orient(a, b, p) == 0
        && p.0 >= a.0.min(b.0)
        && p.0 <= a.0.max(b.0)
        && p.1 >= a.1.min(b.1)
        && p.1 <= a.1.max(b.1)
}

/// Returns whether closed segments `a`-`b` and `c`-`d` share any point
fn touch(a: V, b: V, c: V, d: V) -> bool {
    let (o1, o2) = (orient(a, b, c), orient(a, b, d));
    let (o3, o4) = (orient(c, d, a), orient(c, d, b));
    (o1 * o2 < 0 && o3 * o4 < 0)
        || on_segment(c, a, b)
        || on_segment(d, a, b)
        || on_segment(a, c, d)
        || on_segment(b, c, d)
}

/// Returns whether the segments cross at one point interior to both
fn cross_properly(a: V, b: V, c: V, d: V) -> bool {
    orient(a, b, c) * orient(a, b, d) < 0 && orient(c, d, a) * orient(c, d, b) < 0
}

/// Yields each closed edge as a pair of endpoints
fn edges(outline: &[Point]) -> impl Iterator<Item = (V, V)> + '_ {
    let n = outline.len();
    (0..n).map(move |i| (v(outline[i]), v(outline[(i + 1) % n])))
}

/// Returns whether `p` is inside or on `outline`, all coordinates scaled by `k`
fn contains_scaled(outline: &[Point], p: V, k: i64) -> bool {
    if outline.len() < 3 {
        return false;
    }
    let mut inside = false;
    for (a, b) in edges(outline) {
        let (a, b) = ((a.0 * k, a.1 * k), (b.0 * k, b.1 * k));
        if on_segment(p, a, b) {
            return true;
        }
        // Ray toward +x crosses this edge
        if (a.1 > p.1) != (b.1 > p.1) && (cross(sub(b, a), sub(p, a)) > 0) == (b.1 > a.1) {
            inside = !inside;
        }
    }
    inside
}

/// Returns whether `p` is inside `outline` or on its edge. Always false below 3 points
pub fn contains(outline: &[Point], p: Point) -> bool {
    contains_scaled(outline, v(p), 1)
}

/// Returns `p` if inside `outline`, else its first inside neighbour within one count
pub fn inside_near(outline: &[Point], p: Point) -> Option<Point> {
    let near = |a: u16, d: i8| a.saturating_add_signed(d.into());
    [0, -1, 1]
        .into_iter()
        .flat_map(|dx| [0, -1, 1].map(|dy| (dx, dy)))
        .map(|(dx, dy)| Point {
            x: near(p.x, dx),
            y: near(p.y, dy),
        })
        .find(|&q| contains(outline, q))
}

/// Returns whether every point of segment `a`-`b` is inside or on `outline`
pub fn segment_inside(outline: &[Point], a: Point, b: Point) -> bool {
    if !contains(outline, a) || !contains(outline, b) {
        return false;
    }
    let (va, vb) = (v(a), v(b));
    if edges(outline).any(|(c, d)| cross_properly(va, vb, c, d)) {
        return false;
    }
    // With no proper crossing, the segment meets the boundary only at
    // integer points: its ends and outline vertices on it
    let dir = sub(vb, va);
    let mut cuts = [0i64; 18];
    let mut n = 0;
    for t in [0, dot(dir, dir)].into_iter().chain(
        outline
            .iter()
            .map(|&p| v(p))
            .filter(|&p| on_segment(p, va, vb))
            .map(|p| dot(sub(p, va), dir)),
    ) {
        if n < cuts.len() {
            cuts[n] = t;
            n += 1;
        }
    }
    let cuts = &mut cuts[..n];
    cuts.sort_unstable();
    let point_at = |t: i64| {
        let len2 = dot(dir, dir).max(1);
        (va.0 + dir.0 * t / len2, va.1 + dir.1 * t / len2)
    };
    // Midpoint of each piece in doubled coordinates
    cuts.windows(2).filter(|w| w[0] != w[1]).all(|w| {
        let (p, q) = (point_at(w[0]), point_at(w[1]));
        contains_scaled(outline, (p.0 + q.0, p.1 + q.1), 2)
    })
}

/// Returns whether `outline` touches itself anywhere but between adjacent edges at their shared point
///
/// Two equal points count as crossing, fewer than 3 points otherwise do not
pub fn self_crossing(outline: &[Point]) -> bool {
    let n = outline.len();
    if n == 2 {
        return outline[0] == outline[1];
    }
    if n < 3 {
        return false;
    }
    let at = |i: usize| v(outline[i % n]);
    (0..n).any(|i| {
        let (a, b, c) = (at(i), at(i + 1), at(i + 2));
        // Next edge folding back over this one
        let folds = cross(sub(b, a), sub(c, b)) == 0 && dot(sub(b, a), sub(c, b)) < 0;
        let last = if i == 0 { n - 1 } else { n };
        a == b || folds || (i + 2..last).any(|j| touch(a, b, at(j), at(j + 1)))
    })
}

/// Returns the min and max corners, or `None` for an empty outline
pub fn bbox(outline: &[Point]) -> Option<(Point, Point)> {
    let first = *outline.first()?;
    Some(outline.iter().fold((first, first), |(lo, hi), p| {
        (
            Point {
                x: lo.x.min(p.x),
                y: lo.y.min(p.y),
            },
            Point {
                x: hi.x.max(p.x),
                y: hi.y.max(p.y),
            },
        )
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: u16, y: u16) -> Point {
        Point { x, y }
    }

    const L: [Point; 6] = [
        Point { x: 0, y: 0 },
        Point { x: 100, y: 0 },
        Point { x: 100, y: 100 },
        Point { x: 50, y: 100 },
        Point { x: 50, y: 50 },
        Point { x: 0, y: 50 },
    ];

    fn parse(points: &str) -> Vec<Point> {
        points
            .split(';')
            .filter(|s| !s.is_empty())
            .map(|xy| {
                let (x, y) = xy.split_once(',').unwrap();
                p(x.parse().unwrap(), y.parse().unwrap())
            })
            .collect()
    }

    #[test]
    fn crossing_cases_table() {
        let mut cases = 0;
        for line in include_str!("geometry_cases.txt").lines() {
            if line.starts_with('#') || line.is_empty() {
                continue;
            }
            let (expected, points) = line.split_once('|').unwrap();
            let want = match expected {
                "cross" => true,
                "ok" => false,
                _ => panic!("bad expectation in {line}"),
            };
            assert_eq!(self_crossing(&parse(points)), want, "{line}");
            cases += 1;
        }
        assert!(cases >= 10);
    }

    #[test]
    fn contains_inside_outside_edge() {
        assert!(contains(&L, p(25, 25)));
        assert!(contains(&L, p(75, 75)));
        assert!(!contains(&L, p(25, 75)));
        assert!(contains(&L, p(50, 75)));
        assert!(contains(&L, p(100, 50)));
        assert!(contains(&L, p(0, 0)));
        assert!(!contains(&L, p(101, 50)));
    }

    #[test]
    fn contains_needs_three_points() {
        for n in 0..3 {
            assert!(!contains(&L[..n], p(0, 0)));
        }
    }

    #[test]
    fn segment_inside_l_shape() {
        assert!(segment_inside(&L, p(25, 25), p(75, 25)));
        // Crosses the notch edge at (20, 50)
        assert!(!segment_inside(&L, p(10, 40), p(60, 90)));
        // Ends on both notch edges, cuts the notch without a proper crossing
        assert!(!segment_inside(&L, p(25, 50), p(50, 75)));
    }

    #[test]
    fn segment_through_reflex_corner() {
        // Touches the reflex corner (50, 50) and stays inside
        assert!(segment_inside(&L, p(0, 0), p(100, 100)));
        // C-shape: leaves the reflex corner (30, 30) into the open notch
        let c = [
            p(0, 0),
            p(100, 0),
            p(100, 30),
            p(30, 30),
            p(30, 70),
            p(100, 70),
            p(100, 100),
            p(0, 100),
        ];
        assert!(!segment_inside(&c, p(30, 30), p(80, 70)));
        assert!(segment_inside(&c, p(30, 10), p(30, 90)));
    }

    #[test]
    fn segment_along_edge_inside() {
        assert!(segment_inside(&L, p(0, 0), p(100, 0)));
        assert!(segment_inside(&L, p(50, 100), p(50, 50)));
    }

    #[test]
    fn bbox_of_l() {
        assert_eq!(bbox(&L), Some((p(0, 0), p(100, 100))));
        assert_eq!(bbox(&[]), None);
    }

    #[test]
    fn inside_near_snaps_one_count() {
        let tri = [p(0, 0), p(10, 0), p(10, 3)];
        assert_eq!(inside_near(&tri, p(5, 1)), Some(p(5, 1)));
        let snapped = inside_near(&tri, p(2, 1)).expect("a neighbour is inside");
        assert!(contains(&tri, snapped));
        assert!(snapped.x.abs_diff(2) <= 1 && snapped.y.abs_diff(1) <= 1);
        assert_eq!(inside_near(&tri, p(0, 9)), None);
    }
}
