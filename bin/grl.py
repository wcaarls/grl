#!/usr/bin/python

import yaml
from Tkinter import *
from ttk import *
from functools import partial

# MOVE TO LIB

import yaml
import yaml.constructor

try:
    # included in standard lib from Python 2.7
    from collections import OrderedDict
except ImportError:
    # try importing the backported drop-in replacement
    # it's available on PyPI
    from ordereddict import OrderedDict

# https://gist.github.com/enaeseth/844388
class OrderedDictYAMLLoader(yaml.Loader):
    """
    A YAML loader that loads mappings into ordered dictionaries.
    """

    def __init__(self, *args, **kwargs):
        yaml.Loader.__init__(self, *args, **kwargs)

        self.add_constructor(u'tag:yaml.org,2002:map', type(self).construct_yaml_map)
        self.add_constructor(u'tag:yaml.org,2002:omap', type(self).construct_yaml_map)

    def construct_yaml_map(self, node):
        data = OrderedDict()
        yield data
        value = self.construct_mapping(node)
        data.update(value)

    def construct_mapping(self, node, deep=False):
        if isinstance(node, yaml.MappingNode):
            self.flatten_mapping(node)
        else:
            raise yaml.constructor.ConstructorError(None, None,
                'expected a mapping node, but found %s' % node.id, node.start_mark)

        mapping = OrderedDict()
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            try:
                hash(key)
            except TypeError, exc:
                raise yaml.constructor.ConstructorError('while constructing a mapping',
                    node.start_mark, 'found unacceptable key (%s)' % exc, key_node.start_mark)
            value = self.construct_object(value_node, deep=deep)
            mapping[key] = value
        return mapping

def isobject(type):
  """Returns true if the type is not a builtin type."""
  if type in ['int','double','string','vector']:
    return False
  else:
    return True

def findrequests(type):
  """Find parameter requests that match a certain type."""
  matches = list()
  
  for key in requests:
    if key[0:len(type)] == type:
      matches.append(key)
  
  return matches

def findparams(type):
  """Find registered parameters that match a certain type."""
  matches = list()

  for key in params:
    if params[key][0:len(type)] == type:
      matches.append(key)

  return matches

# MAIN

class GrlMain:
  def __init__(self, master):
    self.row = 0

    self.canvas = Canvas(master, borderwidth=0)
    self.frame = Frame(self.canvas)
    self.vsb = Scrollbar(master, orient="vertical", command=self.canvas.yview)
    self.canvas.configure(yscrollcommand=self.vsb.set)

    self.vsb.pack(side="right", fill="y")
    self.canvas.pack(side="left", fill="both", expand=True)
    self.canvas.create_window((4,4), window=self.frame, anchor="nw", 
                              tags="self.frame")

    self.frame.bind("<Configure>", self.OnFrameConfigure)

    self.runbutton = Button(
          self.frame, text="Run", command=self.run
          )
    self.runbutton.grid(column=0, row=99, columnspan=2, sticky=SW+E)
    self.addbutton = Button(
          self.frame, text="+", command=self.add
          )
    self.addbutton.grid(column=2, row=99, sticky=S)
    
    self.objlist = list()
    self.add()
          
  def OnFrameConfigure(self, event):
    '''Reset the scroll region to encompass the inner frame'''
    self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    self.canvas['width'] = self.frame.winfo_width()
    self.canvas['height'] = self.frame.winfo_height()

  def remove(self, child):
    child.destroy()
    self.objlist.remove(child)

  def run(self):
    self.write()
    
  def add(self):
    obj = GrlTopObject(self, self.row)
    self.objlist.append(obj)
    self.row = self.row + 1
    
    self.refresh()
    
  def refresh(self):
    global params
    params = dict()
  
    for obj in self.objlist:
      obj.refresh()
    
  def write(self):
    for obj in self.objlist:
      obj.write(0)
  
class GrlTopObject:
  def __init__(self, parent, row):
    self.entry = Entry(parent.frame, width=10)
    self.entry.grid(row=row, column=0, sticky=NW)
    if row == 0:
      self.entry.insert(0, "experiment")
    self.frame = Frame(parent.frame)
    self.frame.grid(row=row, column=1, sticky=NW+E)
    self.frame.grid_columnconfigure(0, weight=1)
    self.obj = GrlObject(self, {'type': '', 'description':'Top-level object', 'optional':0})
    self.removebutton = Button(
      parent.frame, text="-", command=partial(parent.remove, self)
      )
    self.removebutton.grid(row=row, column=2, sticky=S)
    
  def change(self, event):
    return True
    
  def refresh(self):
    self.obj.refresh(self.entry.get())
  
  def destroy(self):
    self.removebutton.grid_forget()
    self.removebutton.destroy()
    self.obj.destroy()
    self.frame.grid_forget()
    self.frame.destroy()
    self.entry.grid_forget()
    self.entry.destroy()
    
  def write(self, indent):
    print ''.ljust(indent) + self.entry.get() + ":",
    self.obj.write(indent+2)

class GrlSubObject:
  def __init__(self, parent, name, spec, row):
    self.name = name
    self.label = Label(parent.frame, text=name)
    self.label.grid(column=0, row=row, sticky=NW)
    self.frame = Frame(parent.frame)
    self.frame.grid(column=1, row=row, sticky=NW+E)
    self.frame.grid_columnconfigure(0, weight=1)
    self.obj = GrlObject(self, spec)
    
  def refresh(self, path):
    self.obj.refresh(path + "/" + self.name)
    
  def destroy(self):
    self.obj.destroy()
    self.frame.grid_forget()
    self.frame.destroy()
    self.label.grid_forget()
    self.label.destroy()
    
  def write(self, indent):
    print ''.ljust(indent) + self.label.cget("text") + ":",
    self.obj.write(indent+2)
  
class GrlObject:
  def __init__(self, parent, spec):
    self.spec = spec
    self.type = Combobox(parent.frame, state="readonly", width=40)
    self.type.bind('<<ComboboxSelected>>', self.select)
    self.type.grid(column=0, row=0, sticky=W+E)
    self.frame = Frame(parent.frame)
    self.frame.grid(column=0, row=1, sticky=W+E)
    self.frame.grid_columnconfigure(1, weight=1)
    self.objlist = list()
    
  def refresh(self, path):
    values = findrequests(self.spec["type"])
    values.extend(findparams(self.spec["type"]))
    self.type['values'] = values
  
    for obj in self.objlist:
      obj.refresh(path)

    params[path] = self.type.get()
    
  def destroy(self):
    self.type.grid_forget()
    self.type.destroy()
    self.frame.grid_forget()
    self.frame.destroy()
    
  def select(self, value):
    for obj in self.objlist:
      obj.destroy()
    self.objlist = list()

    type = self.type.get()
    
    row = 0
    if type in requests:
      if requests[type]:
        for key in requests[type]:
          row = row + 1
          if isobject(requests[type][key]["type"]):
            obj = GrlSubObject(self, key, requests[type][key], row)
          else:
            obj = GrlVariable(self, key, requests[type][key], row)
          self.objlist.append(obj)
        
    app.refresh()
    
  def write(self, indent):
    if self.type.get() in requests:
      print '\n' + ''.ljust(indent) + "type: " + self.type.get()
      for obj in self.objlist:
        obj.write(indent)
    else:
      print self.type.get()
    
class GrlVariable:
  def __init__(self, parent, name, spec, row):
    self.spec = spec
    self.name = name
    
    self.label = Label(parent.frame, text=name)
    self.label.grid(column=0, row=row, sticky=NW)
    self.frame = Frame(parent.frame)
    self.frame.grid(column=1, row=row, sticky=NW+E)

    self.value = Combobox(self.frame, width=40)
  
    if spec["type"] == 'int' and spec["max"]-spec["min"] < 10:
      self.value.set(spec["default"])
      self.value.grid(sticky=W+E)
      self.frame.grid_columnconfigure(0, weight=1)
    elif spec["type"] == 'double' and spec["max"]-spec["min"] < 1.0001:
      self.value.set(spec["default"])
      self.value.grid(column=0, row=0)
      self.scale = Scale(self.frame, from_=spec["min"], to=spec["max"], value=spec["default"], command=self.change)
      self.scale.grid(column=1, row=0, sticky=W+E)
      self.frame.grid_columnconfigure(1, weight=1)
    elif spec["type"] == 'string' and "options" in spec and len(spec["options"]) > 0:
      self.value.set(spec["default"])
      self.value.grid(sticky=W+E)
      self.frame.grid_columnconfigure(0, weight=1)
    else:
      self.value.set(spec["default"])
      self.value.grid(sticky=W+E)
      self.frame.grid_columnconfigure(0, weight=1)
      
  def change(self, value):
    self.value.delete(0, END)
    self.value.insert(0, value)
    
  def refresh(self, path):
    if self.spec["type"] == 'int' and self.spec["max"]-self.spec["min"] < 10:
      values = range(self.spec["min"], self.spec["max"]+1)
      values.extend(findparams(self.spec["type"]))
      self.value['values'] = values
    elif self.spec["type"] == 'double' and self.spec["max"]-self.spec["min"] < 1.0001:
      self.value['values'] = findparams(self.spec["type"])
    elif self.spec["type"] == 'string' and "options" in self.spec and len(self.spec["options"]) > 0:
      values = self.spec["options"]
      values.extend(findparams(self.spec["type"]))
      self.value['values'] = values
    else:
      self.value['values'] = findparams(self.spec["type"])

    params[path + "/" + self.name] = self.spec["type"]

  def destroy(self):
    self.label.grid_forget()
    self.label.destroy()
    self.value.grid_forget()
    self.value.destroy()
    # Conditionals
    self.frame.grid_forget()
    self.frame.destroy()
      
  def write(self, indent):
    if len(self.value.get()):
      print ''.ljust(indent) + self.label.cget("text") + ": " + self.value.get()

# Load object parameter requests, generated by requestgen
stream = file('requests.yaml', 'r')
requests = yaml.load(stream, OrderedDictYAMLLoader)
params = dict()
spec = {'type': '', 'description':'Experiment to run', 'optional':0}

root = Tk()
#root.resizable(0,1)
root.title('GRL configurator')

app = GrlMain(root)

root.mainloop()
root.destroy()
