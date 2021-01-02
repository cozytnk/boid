
/**
 * vue instance
 */
const app = new Vue({
  el: '#app',
  data: {
    debug: false,
    p: '',
    boids: [],
    Separation: true,
    Alignment : true,
    Cohesion  : true,
    Convection: false,
  },
  computed: {
  },
  mounted () {
  },
  methods: {
  },
})



/**
 * p5
 */

p5.Vector.prototype.$range = function (min, max) {
  const magSq = this.magSq()
  return (magSq < min) ? this.setMag(min) : (max < magSq) ? this.setMag(max) : this
}

p5.Vector.prototype.$lowerBound = function (x, y) {
  if (this.x < x) this.x = x
  if (this.y < y) this.y = y
  return this
}


// REF: https://t-pot.com/program/101_boid1/index.html
class Boid {
  constructor () {
    this.p = p5.Vector.random2D().mult(100)
    this.v = p5.Vector.random2D()
    this.f = createVector()
    // this.m = 1
    this.m = random(1, 2)

    this.SeparationDist = 30
  }

  draw () {
    push()
    translate(width/2, height/2)
    translate(this.p)
    scale(Math.sqrt(this.m))
    rotate(this.v.heading())
    fill('cornflowerblue')
    noStroke()
    triangle(10, 0, -2, 2, -2, -2)

    if (app.debug) {
      noFill()
      stroke('red')
      strokeWeight(.3)
      circle(0, 0, this.SeparationDist)
    }
    pop()
  }

  update (boids) {
    // const f = createVector(noise(frameCount/100), noise(frameCount/1000))
    // const f = p5.Vector.random2D()
    // this.v.add(this.f.div(this.m)).normalize().mult(2)
    this.v.add(this.f.div(this.m)).$range(1, 3)
    this.p.add(this.v)

    // if (this.p.x < -width /2) this.p.x = width /2
    // if (this.p.y < -height/2) this.p.y = height/2
    // if (width /2 < this.p.x) this.p.x = -width /2
    // if (height/2 < this.p.y) this.p.y = -height/2

    if (this.p.x < -width /2) this.p.x = -width /2
    if (this.p.y < -height/2) this.p.y = -height/2
    if (width /2 < this.p.x) this.p.x = width /2
    if (height/2 < this.p.y) this.p.y = height/2

    // if (this.p.x < -width /2) {
    //   this.p.x = -width /2
    //   this.v.x *= -1
    // }
    // if (this.p.y < -height/2) {
    //   this.p.y = -height/2
    //   this.v.y *= -1
    // }
    // if (width /2 < this.p.x) {
    //   this.p.x = width /2
    //   this.v.x *= -1
    // }
    // if (height/2 < this.p.y) {
    //   this.p.y = height/2
    //   this.v.y *= -1
    // }

  }

  simulate (boids) {
    const f = createVector()
    if(app.Separation) f.add(this.Separation(boids, this.SeparationDist * Math.sqrt(this.m)))
    if(app.Alignment ) f.add(this.Alignment(boids))
    if(app.Cohesion  ) f.add(this.Cohesion(boids))
    f.add(p5.Vector.random2D().mult(0.01))

    // const c = 10 * this.v.magSq()
    const c = 1
    // left wall
    let d = this.p.x - (-width/2)
    f.x += d < 50 ? c / (d ** 2 + 1e-6) : 0
    // right wall
    d = width/2 - this.p.x
    f.x -= d < 50 ? c / (d ** 2 + 1e-6) : 0
    // bottom wall
    d = this.p.y - (-height/2)
    f.y += d < 50 ? c / (d ** 2 + 1e-6) : 0
    // top wall
    d = height/2 - this.p.y
    f.y -= d < 50 ? c / (d ** 2 + 1e-6) : 0

    // f.sub(this.v.copy().mult(-1))
    // f.sub(this.v.copy().normalize().mult(-1))

    // 対流
    // https://t-pot.com/program/102_boid2/index.html
    // https://oshiete.goo.ne.jp/qa/9035497.html
    if (app.Convection) {
      const a = this.p.x,
            b = this.p.y,
            cos = this.p.copy().sub(width/2, height/2).normalize().x,
            sin = this.p.copy().sub(width/2, height/2).normalize().y
      const direction = createVector(b*cos, -a*sin).mult(0.2)
      f.add(direction)
    }

    // f.limit(1)
    this.f = f
  }

  otherBoidsInSight (boids, thr) {
    return boids.filter(boid => this.dist(boid) < thr)
  }
  nearest (boids) {
    let nearest = { dist: 999999, boid: null }
    for (const boid of boids) {
      const dist = this.p.dist(boid.p)
      if (dist < 1e-6) continue
      if (dist < nearest.dist) nearest = { dist, boid }
    }
    return nearest.boid
  }

  Separation (boids, SeparationDist) {
    // keep distance
    const nearest = this.nearest(boids)
    const dist = this.p.dist(nearest.p)
    const directionToNearest = p5.Vector.sub(nearest.p, this.p)
    directionToNearest.normalize()
    if (dist < SeparationDist) {
      // too close
      directionToNearest.mult(-SeparationDist/dist)
    } else if (dist > SeparationDist) {
      // too far
      directionToNearest.mult(dist/SeparationDist)
    } else {
      directionToNearest.mult(0)
    }

    return directionToNearest
  }

  Alignment (boids) {
    const direction = createVector()
    boids.forEach(boid => direction.add(boid.v))
    direction.normalize()
    return direction
  }

  Cohesion (boids) {
    const center = createVector()
    boids.forEach(boid => center.add(boid.p))
    center.div(boids.length)
    const directionToCenter = p5.Vector.sub(center, this.p).normalize()
    return directionToCenter
  }
}

const boids = []

const grid = (step=50) => {
  push()
  stroke('#ddd')
  strokeWeight(.2)
  fill('#ddd')
  textAlign(LEFT, TOP)
  for (let x = 0; x < width; x += step) {
    line(x, 0, x, height)
    text(x.toFixed(0).toString(), x, 0)
  }
  textAlign(LEFT, TOP)
  for (let y = 0; y < height; y += step) {
    line(0, y, width, y)
    text(y.toFixed(0).toString(), 0, y)
  }
  pop()
}

function setup () {
  const w = document.querySelector('#p5canvas-wrapper').clientWidth
  const h = document.querySelector('#p5canvas-wrapper').clientHeight
  const canvas = createCanvas(w, h)
  canvas.parent('#p5canvas-wrapper')

  for (let i = 0; i < 30; i++) boids.push(new Boid())
  Vue.set(app, 'boids', boids)
}

function draw () {
  background('midnightblue')
  if (app.debug) grid(50)

  boids.forEach(boid => boid.simulate(boids))
  boids.forEach(boid => boid.update())
  boids.forEach(boid => boid.draw())

  Vue.set(app, 'p', boids[0].p.toString())
}

function windowResized() {
  const w = document.querySelector('#p5canvas-wrapper').clientWidth
  const h = document.querySelector('#p5canvas-wrapper').clientHeight
  resizeCanvas(w, h)
}
