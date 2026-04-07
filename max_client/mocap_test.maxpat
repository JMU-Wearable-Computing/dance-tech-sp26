{
	"patcher" : {
		"fileversion" : 1,
		"appversion" : {
			"major" : 8,
			"minor" : 3,
			"revision" : 5,
			"architecture" : "x64",
			"modernui" : 1
		},
		"classnamespace" : "box",
		"rect" : [ 100.0, 100.0, 800.0, 400.0 ],
		"bglayer" : 0,
		"openinpresentation" : 0,
		"default_fontsize" : 12.0,
		"default_fontface" : 0,
		"default_fontname" : "Arial",
		"gridonset" : 0,
		"gridsize" : 15.0,
		"gridsnaponopen" : 0,
		"objectsnaponopen" : 1,
		"statusbarvisible" : 2,
		"toolbarvisible" : 1,
		"lefttoolbarpinned" : 0,
		"toptoolbarpinned" : 0,
		"righttoolbarpinned" : 0,
		"bottomtoolbarpinned" : 0,
		"toolbarsvisible" : 1,
		"showinhtmldoc" : 1,
		"showontsp" : 0,
		"enablehscroll" : 1,
		"enablevscroll" : 1,
		"devicewidth" : 0.0,
		"description" : "",
		"digest" : "",
		"tags" : "",
		"style" : "",
		"subpatcher_template" : "",
		"boxes" : [
			{
				"box" : {
					"id" : "obj-1",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 50.0, 20.0, 600.0, 20.0 ],
					"text" : "Mocap OSC Listener - Port 9999. Check Max Console (Ctrl+B) for incoming messages"
				}
			},
			{
				"box" : {
					"id" : "obj-2",
					"maxclass" : "udpreceive",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 50.0, 60.0, 150.0, 22.0 ],
					"args" : [ 9999 ]
				}
			},
			{
				"box" : {
					"id" : "obj-3",
					"maxclass" : "print",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 50.0, 110.0, 100.0, 22.0 ]
				}
			},
			{
				"box" : {
					"id" : "obj-4",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 50.0, 170.0, 600.0, 150.0 ],
					"text" : "This patcher listens on UDP port 9999 for OSC messages.\n\nTo test:\n1. Run: python -m max_client.main\n2. Open Max console: Ctrl+B (Windows) or Cmd+B (Mac)\n3. You should see OSC messages print to console like:\n   /audio/param/chest/quat/raw 0.5 0.2 0.1 0.9\n\nIf you see nothing in console, check max_client.main config:\n   MAXMSP_IP = \"127.0.0.1\"\n   MAXMSP_PORT = 9999"
				}
			}
		],
		"lines" : [
			{
				"patchline" : {
					"source" : [ "obj-2", 0 ],
					"destination" : [ "obj-3", 0 ],
					"hidden" : 0,
					"midpoints" : [  ]
				}
			}
		]
	}
}
