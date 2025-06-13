import type { Object3D } from 'three'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'
import { AmbientLight, DirectionalLight, Matrix4, Vector3 } from 'three'
import camera from './core/camera'
import { fpsGraph, gui } from './core/gui'
import { controls } from './core/orbit-control'
import { renderer, scene } from './core/renderer'
import { Kinematics } from './kinematics'
import './style.css'

const WALL_GLTF_URL = '/models/climbing-holds.glb'
const BEAR_GLTF_URL = '/models/taiwan-bear.glb'

interface Route {
  level: number
  holds: Object3D[]
}

const routes: string[] = [
  '....CFC....', 
  '....F..G...',
  '....F..G...', 
  '...CG..F...',
  '...C.C.F...', 
  '...C.C.....', 
  '..G.C......', 
  '.G..CCC....', 
  '....C.C....',
  '....C.C....',
]

async function loadClimbingWall(gltfUrl: string,routeLines: string[], dx = 0.75, dy = 1.0 ): Promise<[Object3D, Route]> {

  const loader = new GLTFLoader()
  const gltfScene = await new Promise<Object3D>((resolve, reject) => {
    loader.load(gltfUrl, ({ scene }) => resolve(scene), undefined, reject)
  })

  const height = routeLines.length - 1
  const width = routeLines[1].length
  const level = parseInt(routeLines[0].trim(), 10)

  const holds: Object3D[] = []
  const wall = gltfScene.getObjectByName('Cube')!
  wall.children = []

  const jugRight1    = gltfScene.getObjectByName('JugRight1')!
  const jugLeft1     = gltfScene.getObjectByName('JugLeft1')!
  const crimpCenter1 = gltfScene.getObjectByName('CrimpCenter1')!
  const jugCenter2   = gltfScene.getObjectByName('JugCenter2')!
  const markerOn     = gltfScene.getObjectByName('HoldMarkerOn')!
  const markerOff    = gltfScene.getObjectByName('HoldMarkerOff')!

  for (let i = 1; i < routeLines.length; i++) {
    const row = routeLines[i]
    if (!row || row.startsWith('#')) continue

    for (let j = 0; j < row.length; j++) {
      const ch = row[j]
      let proto: Object3D
      let name: string

      switch (ch) {
        case 'C':
          proto = crimpCenter1.clone()
          name  = `CrimpCenter1_${i}_${j}`
          break
        case 'G':
          proto = jugRight1.clone()
          name  = `JugRight1_${i}_${j}`
          break
        case 'F':
          proto = jugLeft1.clone()
          name  = `JugLeft1_${i}_${j}`
          break
        case 'V':
          proto = jugCenter2.clone()
          name  = `JugCenter2_${i}_${j}`
          break
        default:
          continue
      }

      proto.add(markerOn.clone())
      proto.add(markerOff.clone())

      proto.position.set( (width - 1 - j - width / 2) * dx, 0, (height - 1 - (i - 1) - height / 2) * dy)
      proto.name = name
      proto.updateMatrixWorld(true)

      wall.add(proto)
      holds.push(proto)
    }
  }

  return [wall, { level, holds }]
}

function marker_on_off(hold: Object3D, on: boolean) {
  hold.children[0].visible = on
  hold.children[1].visible = !on
}

const ambientLight = new AmbientLight(0xFFFFFF, 0.5)
scene.add(ambientLight)

const directionalLight = new DirectionalLight('#ffffff', 1)
directionalLight.castShadow = true
directionalLight.shadow.mapSize.set(1024, 1024)
directionalLight.shadow.camera.far = 15
directionalLight.shadow.normalBias = 0.05
directionalLight.position.set(0.25, 2, 2.25)
scene.add(directionalLight)

let wall: Object3D | null = null
let route: Route | null = null

loadClimbingWall(WALL_GLTF_URL, routes, 1.5, 1.5).then(([w, r]) => {
  wall = w
  route = r
  scene.add(wall)
})

let bear: Object3D | null = null
let bearKinematics: Kinematics | null = null

const loader = new GLTFLoader()
loader.load(BEAR_GLTF_URL, (gltf) => {
  bear = gltf.scene.children[0]
  bearKinematics = new Kinematics(bear, false)
  scene.add(bear)
  bear.position.set(0, 1.0, -6.5)
  bear.updateMatrixWorld(true)

  const q = bearKinematics.getCurrentStateConfig()
  q.RJoint_Back_Lower_Z_L = (q.RJoint_Back_Lower_Z_L as number) + Math.PI / 4
  q.RJoint_Back_Lower_Z_R = (q.RJoint_Back_Lower_Z_R as number) - Math.PI / 4
  q.RJoint_Front_Lower_Z_L = (q.RJoint_Front_Lower_Z_L as number) - Math.PI / 8
  q.RJoint_Front_Lower_Z_R = (q.RJoint_Front_Lower_Z_R as number) + Math.PI / 8
  bearKinematics.setConfiguration(q)
})

const DirectionalLightFolder = gui.addFolder({ title: 'Directional Light' })
Object.keys(directionalLight.position).forEach((key) => {
  DirectionalLightFolder.addBinding(
    directionalLight.position,
    key as keyof Vector3,
    { min: -100, max: 100, step: 1 }
  )
})

camera.position.set(0, 20, -5)
camera.updateProjectionMatrix()
controls.update()

async function sleep(ms: number) {
  return new Promise(resolve => setTimeout(resolve, ms))
}

function findClosestHold(pos: Vector3, holds: Object3D[], coM: Vector3, isHead: boolean = false) {
  let min = Infinity
  let minHold = holds[0]
  for (const h of holds) {
    const dist = pos.distanceToSquared(h.position)
    const isValidHeadHold = isHead ? h.position.z > coM.z : true
    if (
      isValidHeadHold &&
      (
        (h.name.includes('Left') && h.position.x >= coM.x) ||
        (h.name.includes('Right') && h.position.x <= coM.x) ||
        h.name.includes('Center')
      )
    ) {
      if (dist < min) {
        min = dist
        minHold = h
      }
    }
  }
  return minHold
}

function hang_on_wall(bearKinematics: Kinematics, holds: Object3D[], setMarkers = false) {
  if (!holds.length) return false

  holds.forEach(h => marker_on_off(h, false))

  let init = false
  const effectors = [
    'Effector_Back_L',
    'Effector_Back_R',
    'Effector_Front_L',
    'Effector_Front_R',
    'Effector_Head'
  ]

  for (const eff of effectors) {
    const q = bearKinematics.getCurrentStateConfig()
    const origin = bearKinematics.root.matrixWorld
    const def_m = bearKinematics.homePositionFwd[eff]
    if (!def_m) continue

    const m = new Matrix4().multiplyMatrices(origin, def_m)
    const current = new Vector3(m.elements[12], m.elements[13], m.elements[14])
    const isHead = eff === 'Effector_Head'
    const h = findClosestHold(current, holds, bearKinematics.root.position, isHead)
    const [newConfig, err] = bearKinematics.inverseKinematics(
      q,
      eff,
      h.position,
      origin,
      isHead ? 0.1 : 0.3,
      0.0001,
      isHead ? 50 : 100
    )
    if (setMarkers && err.lengthSq() <= 0.0001) {
      marker_on_off(h, true)
    }
    bearKinematics.setConfiguration(newConfig)
    init = true
  }

  return init
}

let pathCounter = 0

const loop = async () => {
  fpsGraph.begin()
  renderer.render(scene, camera)

  if (bearKinematics && route && bear) {
    hang_on_wall(bearKinematics, route.holds, true)

    if (pathCounter < 20) {
      bearKinematics.root.translateZ(0.05)
    } else if (pathCounter < 30) {
      bearKinematics.root.translateZ(0.05)
      bearKinematics.root.translateX(0.1)
    } else if (pathCounter < 50) {
      bearKinematics.root.translateZ(0.05)
    } else if (pathCounter < 60) {
      bearKinematics.root.translateZ(0.1)
      bearKinematics.root.translateX(0.05)
    } else if (pathCounter < 80) {
      bearKinematics.root.translateZ(0.1)
    } else if (pathCounter < 120) {
      bearKinematics.root.translateZ(0.1)
      bearKinematics.root.translateX(-0.05)
    } else if (pathCounter < 130) {
      bearKinematics.root.translateZ(0.1)
    }

    if (pathCounter % 10 === 0) {
      const q = bearKinematics.getCurrentStateConfig()
      const origin = bearKinematics.root.matrixWorld
      const headEff = 'Effector_Head'
      const def_m = bearKinematics.homePositionFwd[headEff]
      if (def_m) {
        const m = new Matrix4().multiplyMatrices(origin, def_m)
        const current = new Vector3(m.elements[12], m.elements[13], m.elements[14])
        const nextHold = findClosestHold(
          new Vector3(current.x, current.y, current.z + 1),
          route.holds,
          bearKinematics.root.position,
          true
        )
        const [newConfig] = bearKinematics.inverseKinematics(
          q,
          headEff,
          nextHold.position,
          origin,
          0.1,
          0.0001,
          50
        )
        bearKinematics.setConfiguration(newConfig)
      }
    }

    bearKinematics.root.updateMatrixWorld(true)
    pathCounter++
    await sleep(10)
  }

  fpsGraph.end()
  requestAnimationFrame(loop)
}

loop()
