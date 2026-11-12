package grasppose

import (
	"math"

	"github.com/golang/geo/r3"
)

// blockPoints is a block seen from above, as world points (mm), 2 mm apart:
// its top face at base+height and, optionally, its four sides from base up,
// and a table ring round it at base (120 mm square, outside the block).
func blockPoints(cx, cy, yawDeg, length, width, base, height float64, sides, table bool) []r3.Vector {
	a := yawDeg * math.Pi / 180
	at := func(u, v, z float64) r3.Vector {
		return r3.Vector{X: cx + u*math.Cos(a) - v*math.Sin(a), Y: cy + u*math.Sin(a) + v*math.Cos(a), Z: z}
	}
	var pts []r3.Vector
	for u := -length / 2; u <= length/2; u += 2 {
		for v := -width / 2; v <= width/2; v += 2 {
			pts = append(pts, at(u, v, base+height))
		}
	}
	if sides {
		for z := base + 2; z < base+height; z += 2 {
			for u := -length / 2; u <= length/2; u += 2 {
				pts = append(pts, at(u, -width/2, z), at(u, width/2, z))
			}
			for v := -width / 2; v <= width/2; v += 2 {
				pts = append(pts, at(-length/2, v, z), at(length/2, v, z))
			}
		}
	}
	if table {
		for x := -60.0; x <= 60; x += 2 {
			for y := -60.0; y <= 60; y += 2 {
				if math.Abs(x) > length/2+2 || math.Abs(y) > width/2+2 {
					pts = append(pts, r3.Vector{X: cx + x, Y: cy + y, Z: base})
				}
			}
		}
	}
	return pts
}

func ptr(v float64) *float64 { return &v }
