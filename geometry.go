package grasppose

import (
	"math"
	"sort"

	"github.com/golang/geo/r3"
)

const (
	cellMM    = 5.0 // top-down grid for the own-cluster step
	minPoints = 30  // fewer points left than this: no object
)

// rect is a rectangle seen from above: its centre, the yaw of its long side
// in [-90, 90) degrees, and its length (long side) and width.
type rect struct{ cx, cy, yawDeg, length, width float64 }

// normDeg returns d in [-90, 90).
func normDeg(d float64) float64 { return d - 180*math.Floor((d+90)/180) }

func zs(points []r3.Vector) []float64 {
	out := make([]float64, len(points))
	for i, p := range points {
		out[i] = p.Z
	}
	return out
}

// percentile of values (q in 0..100), linear between ranks.
func percentile(values []float64, q float64) float64 {
	s := append([]float64(nil), values...)
	sort.Float64s(s)
	pos := q / 100 * float64(len(s)-1)
	i := int(math.Floor(pos))
	if i+1 >= len(s) {
		return s[len(s)-1]
	}
	return s[i] + (pos-float64(i))*(s[i+1]-s[i])
}

// dropSupport removes the surface the object stands on: points with
// z <= level + supportMargin, the level being support_z_mm when configured
// (known) or else the points' 2nd percentile of z (estimated). An estimated
// level drops nothing when the points span no more than 2 x supportMargin:
// a top-only view whose table the vision service already removed.
func dropSupport(points []r3.Vector, s settings) (kept []r3.Vector, level float64, known bool) {
	z := zs(points)
	if s.supportZ != nil {
		level, known = *s.supportZ, true
	} else {
		level = percentile(z, 2)
		if percentile(z, 100)-percentile(z, 0) <= 2*s.supportMargin {
			return points, level, false
		}
	}
	for _, p := range points {
		if p.Z > level+s.supportMargin {
			kept = append(kept, p)
		}
	}
	return kept, level, known
}

type cell [2]int

func cellOf(p r3.Vector) cell {
	return cell{int(math.Floor(p.X / cellMM)), int(math.Floor(p.Y / cellMM))}
}

// ownCluster keeps, seen from above on a cellMM grid, the 8-connected cluster
// of occupied cells nearest the points' median: a detection box can take in
// a neighbour's corner.
func ownCluster(points []r3.Vector) []r3.Vector {
	if len(points) == 0 {
		return points
	}
	occupied := map[cell]bool{}
	xs, ys := make([]float64, len(points)), make([]float64, len(points))
	for i, p := range points {
		occupied[cellOf(p)] = true
		xs[i], ys[i] = p.X, p.Y
	}
	mx, my := percentile(xs, 50)/cellMM, percentile(ys, 50)/cellMM
	var start cell
	best := math.Inf(1)
	for c := range occupied {
		d := math.Pow(float64(c[0])+0.5-mx, 2) + math.Pow(float64(c[1])+0.5-my, 2)
		if d < best || (d == best && (c[0] < start[0] || (c[0] == start[0] && c[1] < start[1]))) {
			best, start = d, c
		}
	}
	cluster := map[cell]bool{start: true}
	frontier := []cell{start}
	for len(frontier) > 0 {
		c := frontier[len(frontier)-1]
		frontier = frontier[:len(frontier)-1]
		for dx := -1; dx <= 1; dx++ {
			for dy := -1; dy <= 1; dy++ {
				n := cell{c[0] + dx, c[1] + dy}
				if occupied[n] && !cluster[n] {
					cluster[n] = true
					frontier = append(frontier, n)
				}
			}
		}
	}
	var out []r3.Vector
	for _, p := range points {
		if cluster[cellOf(p)] {
			out = append(out, p)
		}
	}
	return out
}

// minAreaRect is the smallest rectangle round the points seen from above,
// tried a degree at a time over [0, 90) (a square has no main axis, so
// principal axes would turn it anywhere): vision_pick.py's grasp(), ported.
func minAreaRect(points []r3.Vector) rect {
	bestArea := math.Inf(1)
	var bestDeg int
	var bu, bv, cu, cv float64
	for deg := 0; deg < 90; deg++ {
		a := float64(deg) * math.Pi / 180
		ca, sa := math.Cos(a), math.Sin(a)
		umin, umax, vmin, vmax := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
		for _, p := range points {
			u, v := p.X*ca+p.Y*sa, -p.X*sa+p.Y*ca
			umin, umax = math.Min(umin, u), math.Max(umax, u)
			vmin, vmax = math.Min(vmin, v), math.Max(vmax, v)
		}
		if area := (umax - umin) * (vmax - vmin); area < bestArea {
			bestArea, bestDeg = area, deg
			bu, bv, cu, cv = umax-umin, vmax-vmin, (umax+umin)/2, (vmax+vmin)/2
		}
	}
	a := float64(bestDeg) * math.Pi / 180
	r := rect{cx: cu*math.Cos(a) - cv*math.Sin(a), cy: cu*math.Sin(a) + cv*math.Cos(a)}
	long := float64(bestDeg)
	r.length, r.width = bu, bv
	if bu < bv {
		long += 90
		r.length, r.width = bv, bu
	}
	r.yawDeg = normDeg(long)
	return r
}
