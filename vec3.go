package quadcopter

type Vec3 struct {
	X float64
	Y float64
	Z float64
}

func (v *Vec3) Add(n *Vec3) {
	v.X += n.X
	v.Y += n.Y
	v.Z += n.Z
}

func (v *Vec3) Reset() {
	v.X = 0
	v.Y = 0
	v.Z = 0
}

func (v *Vec3) Divide(n float64) {
	v.X /= n
	v.Y /= n
	v.Z /= n
}

func (v *Vec3) Clone() *Vec3 {
	return &Vec3{
		X: v.X,
		Y: v.Y,
		Z: v.Z,
	}
}
