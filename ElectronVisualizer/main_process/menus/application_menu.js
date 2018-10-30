const {Menu, dialog, app} = require('electron')


var template = [
  {
    label: 'File',
    submenu: [
       {
         label: 'Read Script',
         accelerator: 'CmdOrCtrl+R',
		     click: function() {
          dialog.showOpenDialog({
            properties: ['openFile']
          }, (file) => {
            if (file) {
              global.mainWindow.webContents.send('selected_file',file)
            }
          })
        }
       }
     ]
  }]

app.on('ready', () => {
  const menu = Menu.buildFromTemplate(template)
  Menu.setApplicationMenu(menu)
})

