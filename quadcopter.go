package quadcopter

type Quadcopter struct {
	Esc []*ESC
}

func NewQuadcopter() *Quadcopter {
	quad := new(Quadcopter)
	quad.Esc = make([]*ESC, 0)
	return quad
}

func (quad *Quadcopter) GetEsc(idx int) *ESC {
	if idx < len(quad.Esc) {
		return quad.Esc[idx]
	}
	return nil
}
