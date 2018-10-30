const Cesium = require('cesium')
const {ipcRenderer} = require('electron')
const path = require('path')

// Inital Cesium View
global.viewer = null

function InitalCesiumView () {
	viewer = new Cesium.Viewer('cesiumContainer');
	viewer.camera.flyHome(0);
}


function ReadCZML(element, index, array) {
    viewer.dataSources.add(Cesium.CzmlDataSource.load(element));
}

ipcRenderer.on('selected_file', (event, file) => {

	file.forEach(ReadCZML)
})
