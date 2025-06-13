import { Object3D, Vector3, Matrix4 } from 'three'

interface Rot3Angles {
  x: number
  y: number
  z: number
}

interface JointAngles {
  [key: string]: number | Rot3Angles
}

interface NameChildMap {
  [key: string]: Object3D
}

interface LinkTransformations {
  [id: string]: Matrix4
}

function sortObjectKeys<T extends Record<string, any>>(obj: T): T {
  return Object.keys(obj)
    .sort()
    .reduce((sortedObj, key) => {
      (sortedObj as any)[key] = obj[key]
      return sortedObj
    }, {} as T)
}

function isDictionary(obj: unknown): obj is Record<string, unknown> {
  return typeof obj === 'object' && obj !== null && !Array.isArray(obj)
}

class Kinematics {
  root: Object3D
  map: NameChildMap
  homePositionConfig: JointAngles
  homePositionFwd: LinkTransformations
  qConfigIndex: { [key: string]: number }
  qConfigIndexReversed: string[]
  qConfigLength: number
  yup: boolean

  constructor(root: Object3D, yup: boolean = false) {
    this.root = root
    this.yup = yup

    this.map = sortObjectKeys(this.getNameChildMap(this.root))
    const [ind, indRev] = this.confIndex(this.map)
    this.qConfigIndex = ind
    this.qConfigIndexReversed = indRev
    this.qConfigLength = this.qConfigIndexReversed.length
    
    {
      const q = this.getCurrentStateConfig()
      q.RJoint_Back_Lower_Z_L = (q.RJoint_Back_Lower_Z_L as number) + Math.PI / 4
      q.RJoint_Back_Lower_Z_R = (q.RJoint_Back_Lower_Z_R as number) - Math.PI / 4
      q.RJoint_Front_Lower_Z_L = (q.RJoint_Front_Lower_Z_L as number) - Math.PI / 8
      q.RJoint_Front_Lower_Z_R = (q.RJoint_Front_Lower_Z_R as number) + Math.PI / 8
      this.homePositionConfig = q
    }

    this.homePositionFwd = this.forwardKinematics(this.homePositionConfig, new Matrix4())
  }

  confIndex(map: NameChildMap): [{ [key: string]: number }, string[]] {
    const qConfigIndex: { [key: string]: number } = {}
    const qConfigIndexReversed: string[] = []

    let count = 0
    for (const joint in map) {
      if (joint.includes('_XYZ_')) {
        qConfigIndex[`${joint}/x`] = count
        qConfigIndexReversed.push(`${joint}/x`)
        count = count + 1

        qConfigIndex[`${joint}/y`] = count
        qConfigIndexReversed.push(`${joint}/y`)
        count = count + 1

        qConfigIndex[`${joint}/z`] = count
        qConfigIndexReversed.push(`${joint}/z`)
        count = count + 1
      }
      else if (joint.includes('_Z_')) {
        qConfigIndex[joint] = count
        qConfigIndexReversed.push(joint)
        count = count + 1
      }
    }

    return [qConfigIndex, qConfigIndexReversed]
  }

  forwardKinematicsRec(
    root: Object3D,
    q: JointAngles,
    A: Matrix4,
    links: LinkTransformations,
  ): LinkTransformations {
    links[root.name] = root.matrixWorld.clone()
    for (const c of root.children) {
      this.forwardKinematicsRec(c, q, A, links)
    }
    return links
  }

  forwardKinematics(q: JointAngles, A: Matrix4): LinkTransformations {
    const links: LinkTransformations = {}
    const q_old = this.getCurrentStateConfig()
    this.setConfiguration(q)

    this.forwardKinematicsRec(this.root, q, A, links)

    this.setConfiguration(q_old)
    return links
  }

  qConfigStringToIndex(label: string) {
    return this.qConfigIndex[label]
  }

  calcJacobian(
    q: JointAngles,
    joint: string,
    pos: Vector3,
    dq: number,
    A: Matrix4,
  ) {
    const cVec = this.configToVector(q)
    const jac: Array<Vector3> = new Array<Vector3>()

    for (let i = 0; i < this.qConfigLength; i++) {
      const vec = [...cVec]
      vec[i] = vec[i] + dq
      const q2 = this.vectorToConfig(vec)
      const fwd2 = this.forwardKinematics(q2, A)
      const m2 = fwd2[joint]
      const pos2 = this.getPosition(m2)

      const diff = pos2.clone()
      diff.sub(pos)
      diff.divideScalar(dq)

      jac.push(diff)
    }

    return jac
  }

  getPosition(m: Matrix4) {
    const me = m.elements
    return new Vector3(me[0 + 3 * 4], me[1 + 3 * 4], me[2 + 3 * 4])
  }

  getChain(endEffector: string): string[] {
    const chain: string[] = [endEffector]
    let current = this.map[endEffector]?.parent as Object3D
    while (current && current.name !== this.root.name) {
      if (current.name.startsWith('RJoint_')) {
        chain.push(current.name)
      }
      current = current.parent as Object3D
    }
    return chain.reverse() // From base to end-effector
  }

  inverseKinematics(
    q: JointAngles,
    joint: string,
    target: Vector3,
    A: Matrix4,
    alpha: number = 0.5,
    limit: number = 0.0001,
    maxIterations: number = 100,
  ): [JointAngles, Vector3, number, LinkTransformations] {
    let count = 0
    const vec = this.configToVector(q)
    let fwd = this.forwardKinematics(this.vectorToConfig(vec), A)
    let p_end = this.getPosition(fwd[joint])
    let dErr = new Vector3().subVectors(target, p_end)

    const chain = this.getChain(joint)

    while (dErr.lengthSq() > limit && count < maxIterations) {
      for (let j = chain.length - 1; j >= 0; j--) {
        const jointName = chain[j]
        const dofs = jointName.includes('_XYZ_') ? ['x', 'y', 'z'] : ['z']
        
        for (const dof of dofs) {
          const label = jointName.includes('_XYZ_') ? `${jointName}/${dof}` : jointName
          const i = this.qConfigIndex[label]
          if (i === undefined) continue // Skip if DOF not in config

          const matrix = fwd[jointName]
          const me = matrix.elements
          let axis: Vector3
          switch (dof) {
            case 'x':
              axis = new Vector3(me[0], me[1], me[2]).normalize()
              break
            case 'y':
              axis = new Vector3(me[4], me[5], me[6]).normalize()
              break
            case 'z':
              axis = new Vector3(me[8], me[9], me[10]).normalize()
              break
            default:
              continue
          }

          const p_joint = this.getPosition(matrix)
          p_end = this.getPosition(fwd[joint])
          const v_current = new Vector3().subVectors(p_end, p_joint)
          const v_target = new Vector3().subVectors(target, p_joint)

          const v_curr_proj = v_current.clone().sub(
            axis.clone().multiplyScalar(v_current.dot(axis))
          )
          const v_targ_proj = v_target.clone().sub(
            axis.clone().multiplyScalar(v_target.dot(axis))
          )

          if (v_curr_proj.lengthSq() < 0.0001 || v_targ_proj.lengthSq() < 0.0001) {
            continue
          }

          v_curr_proj.normalize()
          v_targ_proj.normalize()
          const dot = Math.max(-1, Math.min(1, v_curr_proj.dot(v_targ_proj)))
          const cross = v_curr_proj.cross(v_targ_proj)
          const theta = Math.atan2(cross.dot(axis), dot)
          
          vec[i] += alpha * theta

          fwd = this.forwardKinematics(this.vectorToConfig(vec), A)
          p_end = this.getPosition(fwd[joint])
          dErr = new Vector3().subVectors(target, p_end)
        }
      }
      count++
    }

    const finalQ = this.vectorToConfig(vec)
    return [finalQ, dErr, count, fwd]
  }

  getNameChildMap(obj: Object3D): NameChildMap {
    const map: NameChildMap = {}
    obj.traverse((child) => {
      if (child instanceof Object3D) {
        const name = child.name
        map[name] = child
      }
    })
    return map
  }

  configToVector(q: JointAngles) {
    const vec: number[] = Array.from({ length: this.qConfigLength }).fill(-1000) as number[]
    for (const joint in q) {
      const qv = q[joint]
      if (isDictionary(qv)) {
        const qc = qv as Rot3Angles
        vec[this.qConfigIndex[`${joint}/x`]] = qc.x
        vec[this.qConfigIndex[`${joint}/y`]] = qc.y
        vec[this.qConfigIndex[`${joint}/z`]] = qc.z
      }
      else {
        vec[this.qConfigIndex[joint]] = qv as number
      }
    }
    return vec
  }

  vectorToConfig(vec: number[]) {
    const q: JointAngles = {}
    for (let i = 0; i < vec.length; i++) {
      const jLabel = this.qConfigIndexReversed[i]
      if (jLabel.endsWith('/x')) {
        if (q[jLabel.substring(0, jLabel.length - 2)] === undefined) {
          q[jLabel.substring(0, jLabel.length - 2)] = {} as Rot3Angles
        }
        (q[jLabel.substring(0, jLabel.length - 2)] as Rot3Angles).x = vec[i]
      }
      else if (jLabel.endsWith('/y')) {
        if (q[jLabel.substring(0, jLabel.length - 2)] === undefined) {
          q[jLabel.substring(0, jLabel.length - 2)] = {} as Rot3Angles
        }
        (q[jLabel.substring(0, jLabel.length - 2)] as Rot3Angles).y = vec[i]
      }
      else if (jLabel.endsWith('/z')) {
        if (q[jLabel.substring(0, jLabel.length - 2)] === undefined) {
          q[jLabel.substring(0, jLabel.length - 2)] = {} as Rot3Angles
        }
        (q[jLabel.substring(0, jLabel.length - 2)] as Rot3Angles).z = vec[i]
      }
      else {
        q[jLabel] = vec[i]
      }
    }
    return q
  }

  getCurrentStateVector() {
    return this.configToVector(this.getCurrentStateConfig())
  }

  getCurrentStateConfig() {
    const q: JointAngles = {}
    for (const name in this.map) {
      if (name.startsWith('RJoint_')) {
        const child = this.map[name]
        if (child.rotation && child.rotation.isEuler) {
          if (name.includes('_XYZ_')) {
            q[name] = {
              x: child.rotation.x,
              y: child.rotation.y,
              z: child.rotation.z,
            }
          }
          else if (name.includes('_Z_')) {
            q[name] = child.rotation.z
          }
        }
      }
    }
    return q
  }

  setConfiguration(q: JointAngles) {
    for (const joint in q) {
      const child = this.map[joint]
      const qv = q[joint]

      if (isDictionary(qv)) {
        const qc = qv as Rot3Angles
        child.rotation.set(qc.x, qc.y, qc.z, 'XYZ')
      }
      else {
        if (!this.yup) {
          child.rotation.set(
            child.rotation.x,
            child.rotation.y,
            qv as number,
            'XYZ',
          )
        }
        else {
          child.rotation.set(
            child.rotation.x,
            qv as number,
            child.rotation.z,
            'XYZ',
          )
        }
      }
      child.updateMatrixWorld(true)
    }
  }
}

export { Kinematics }