define((require, exports, module) => {
  const gui = require('UiDat')
  const storeManager = require('State')
  const robotStore = require('Robot')

  const geometry = storeManager.getStore('Robot').getState().geometry
  const jointLimits = storeManager.getStore('Robot').getState().jointLimits

  const robotGuiStore = storeManager.createStore('RobotGui', {})


  const DEG_TO_RAD = Math.PI / 180
  const RAD_TO_DEG = 180 / Math.PI
  /* DAT GUI */

  const anglesDeg = {
    A0: 0,
    A1: 0,
    A2: 0,
    A3: 0,
    A4: 0,
    A5: 0,
  }


  const jointLimitsDeg = {
    J0: [-190, 190],
    J1: [-58, 90],
    J2: [-135, 40],
    J3: [-90, 75],
    J4: [-139, 20],
    J5: [-188, 181],
  }

  robotStore.listen([state => state.angles], (angles) => {
    Object.keys(anglesDeg).forEach((k) => {
      anglesDeg[k] = angles[k] / Math.PI * 180
    })
  })

  const anglesGui = gui.addFolder('angles')
  let i = 0
  for (const key in anglesDeg) {
    anglesGui.add(anglesDeg, key).min(jointLimits[`J${i}`][0] * RAD_TO_DEG).max(jointLimits[`J${i++}`][1] * RAD_TO_DEG).step(1).listen().onChange(() => {
      const anglesRad = {}
      for (const key in anglesDeg) {
        if (anglesDeg.hasOwnProperty(key)) {
          anglesRad[key] = anglesDeg[key] * DEG_TO_RAD
        }
      }
      robotStore.dispatch('ROBOT_CHANGE_ANGLES', anglesRad)
    })
  }


  /* END DAT GUI */

  module.exports = robotStore
})
