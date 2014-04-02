#!/usr/bin/env python

from __future__ import print_function

import itertools
import os
import argparse
import xml.etree.ElementTree as ET
import xml.dom.minidom
from tf.transformations import *

def prettyXML(uglyXML):
  return xml.dom.minidom.parseString(uglyXML).toprettyxml(indent='  ')

def pose2origin(pose):
  xyz = ' '.join(pose.split()[:3])
  rpy = ' '.join(pose.split()[3:])
  return xyz, rpy

def pose2tf(pose):
  pose_float = [float(i) for i in pose.split()]
  return compose_matrix(None, None, pose_float[3:], pose_float[:3])

def tf2pose(tf):
  scale, shear, angles, trans, persp = decompose_matrix(tf)
  return ' '.join(str(i) for i in itertools.chain(trans, angles))

def pose_multiply(pose1, pose2):
  #print('pose_multiply(%s, %s)' % (pose1, pose2), end='')
  pose1_tf = pose2tf(pose1)
  pose2_tf = pose2tf(pose2)
  pose_tf = numpy.dot(pose1_tf, pose2_tf)
  pose = tf2pose(pose_tf)
  #print(' -> %s' % (pose))
  return pose

def tf_multiply(tf1, tf2):
  return numpy.dot(tf1, tf2)



class Link:
  def __init__(self, link_tag, prefix = ''):
    self.name = link_tag.attrib['name']
    if prefix:
      self.name = prefix + '::' + self.name
    self.pose = None
    self.inertial = {}
    self.collision = {}
    self.visual = {}

    pose_tag = link_tag.find('pose')
    if pose_tag != None:
      self.pose = pose_tag.text.replace('\n', ' ').strip()

    inertial = link_tag.find('inertial')
    if inertial != None:
      pose_tag = inertial.find('pose')
      if pose_tag != None:
        self.inertial['pose'] = pose_tag.text.replace('\n', ' ').strip()
      mass_tag = inertial.find('mass')
      if mass_tag != None:
        self.inertial['mass'] = mass_tag.text
      inertia_tag = inertial.find('inertia')
      if inertia_tag != None:
        inertia_vals = {}
        for coord in 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz':
          coord_val_tag = inertia_tag.find(coord)
          if coord_val_tag != None:
            inertia_vals[coord] = coord_val_tag.text
        self.inertial['inertia'] = inertia_vals

    for elem in 'collision', 'visual':
      elem_tag = link_tag.find(elem)
      if elem_tag != None:
        getattr(self, elem)['name'] = elem_tag.attrib['name']
        pose = elem_tag.find('pose')
        if pose != None:
          getattr(self, elem)['pose'] = pose.text
        geometry = elem_tag.find('geometry')
        if geometry != None:
          geometry_vals = {}
          sphere = geometry.find('sphere')
          if sphere != None:
            radius = sphere.find('radius')
            geometry_vals['sphere'] = {'radius': radius.text}
          cylinder = geometry.find('cylinder')
          if cylinder != None:
            radius = cylinder.find('radius')
            length = cylinder.find('length')
            geometry_vals['cylinder'] = {'radius': radius.text, 'length': length.text}
          box = geometry.find('box')
          if box != None:
            size = box.find('size')
            geometry_vals['box'] = {'size': size.text}
          mesh = geometry.find('mesh')
          if mesh != None:
            uri = mesh.find('uri')
            mesh_vals = {'uri': uri.text}
            scale = mesh.find('scale')
            if scale != None:
              mesh_vals['scale'] = scale.text
            geometry_vals['mesh'] = mesh_vals
          getattr(self, elem)['geometry'] = geometry_vals


  def toUrdfSubElement(self, parent_tag):
    link_tag = ET.SubElement(parent_tag, 'link', {'name': self.name})
    for elem in 'collision', 'visual':
      if getattr(self, elem):
        elem_tag = ET.SubElement(link_tag, elem, {'name': getattr(self, elem)['name']})
        if 'pose' in getattr(self, elem):
          xyz, rpy = pose2origin(self.pose)
          origin_tag = ET.SubElement(elem_tag, 'origin', {'rpy': rpy, 'xyz': xyz})
        if 'geometry' in getattr(self, elem):
          geometry_tag = ET.SubElement(elem_tag, 'geometry')
          if 'mesh' in getattr(self, elem)['geometry']:
            mesh_tag = ET.SubElement(geometry_tag, 'mesh', {'filename':  'package://PATHTOMESHES/' + '/'.join(getattr(self, elem)['geometry']['mesh']['uri'].split('/')[3:])})
          if 'sphere' in getattr(self, elem)['geometry']:
            sphere_tag = ET.SubElement(geometry_tag, 'sphere', {'radius': getattr(self, elem)['geometry']['sphere']['radius']})
          if 'cylinder' in getattr(self, elem)['geometry']:
            cylinder_tag = ET.SubElement(geometry_tag, 'cylinder', {'radius': getattr(self, elem)['geometry']['cylinder']['radius'], 'length': getattr(self, elem)['geometry']['cylinder']['length']})
          if 'box' in getattr(self, elem)['geometry']:
            box_tag = ET.SubElement(geometry_tag, 'box', {'size': getattr(self, elem)['geometry']['box']['size']})
    if self.inertial:
      inertial_tag = ET.SubElement(link_tag, 'inertial')
      mass_tag = ET.SubElement(inertial_tag, 'mass', {'value': self.inertial['mass']})
      if 'pose' in self.inertial:
        xyz, rpy = pose2origin(self.inertial['pose'])
        origin_tag = ET.SubElement(inertial_tag, 'origin', {'rpy': rpy, 'xyz': xyz})
      inertia_tag = ET.SubElement(inertial_tag, 'inertia')
      for coord in 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz':
        if 'inertia' in self.inertial:
          inertia = self.inertial['inertia']
        else:
          inertia = {}
        inertia_tag.attrib[coord] = inertia.get(coord, '0')

  def __repr__(self):
    return 'Link(name=%s, pose=%s, inertial=%s, collision=%s, visual=%s)' % (self.name, self.pose, str(self.inertial), str(self.collision), str(self.visual))




class Joint:
  def __init__(self, joint_tag, prefix = ''):
    self.name = joint_tag.attrib['name']
    self.joint_type = joint_tag.attrib['type']
    self.child = joint_tag.find('child').text
    self.parent = joint_tag.find('parent').text
    if prefix:
      self.name = prefix + '::' + self.name
      self.child = prefix + '::' + self.child
      self.parent = prefix + '::' + self.parent
    self.pose = None
    self.axis = {}

    pose_tag = joint_tag.find('pose')
    if pose_tag != None:
      self.pose = pose_tag.text.replace('\n', ' ').strip()
    axis_tag = joint_tag.find('axis')
    xyz_tag = axis_tag.find('xyz')
    if xyz_tag != None:
      self.axis['xyz'] = xyz_tag.text
    limit_tag = axis_tag.find('limit')
    if limit_tag != None:
      limit_vals = {}
      for elem in 'lower', 'upper':
        elem_tag = limit_tag.find(elem)
        if elem_tag != None:
          limit_vals[elem] = elem_tag.text
        else:
          print('ERROR: <joint><limit> is missing <%s> tag, which is required for URDF' % elem)
      for elem in 'effort', 'velocity':
        elem_tag = limit_tag.find(elem)
        if elem_tag != None:
          limit_vals[elem] = elem_tag.text
        elif not (self.joint_type == 'revolute' and float(limit_vals['lower']) == 0 and float(limit_vals['upper']) == 0):
          print('ERROR: <joint><limit> is missing <%s> tag (of a non-fixed joint), which is required for URDF' % elem)
      self.axis['limit'] = limit_vals


  def toUrdfSubElement(self, parent_tag):
    joint_tag = ET.SubElement(parent_tag, 'joint', {'name': self.name})
    if self.joint_type == 'revolute' and 'limit' in self.axis and float(self.axis['limit']['lower']) == 0 and float(self.axis['limit']['upper']) == 0:
      joint_tag.attrib['type'] = 'fixed'
    else:
      joint_tag.attrib['type'] = self.joint_type

    parent_tag = ET.SubElement(joint_tag, 'parent', {'link': self.parent})
    child_tag = ET.SubElement(joint_tag, 'child', {'link': self.child})

    if self.pose:
      xyz, rpy = pose2origin(self.pose)
      origin_tag = ET.SubElement(joint_tag, 'origin', {'rpy': rpy, 'xyz': xyz})

    if self.axis:
      axis_tag = ET.SubElement(joint_tag, 'axis')
      if 'xyz' in self.axis:
        axis_tag.attrib['xyz'] = self.axis['xyz']
    if 'limit' in self.axis and joint_tag.attrib['type'] != 'fixed':
      limit_tag = ET.SubElement(joint_tag, 'limit')
      for attrib in 'lower', 'upper', 'effort', 'velocity':
        if attrib in self.axis['limit']:
          limit_tag.attrib[attrib] = self.axis['limit'][attrib]


  def __repr__(self):
    return 'Joint(name=%s, type=%s, child=%s, parent=%s, axis=%s, pose=%s)' % (self.name, self.joint_type, self.child, self.parent, str(self.axis), self.pose)




class Model:
  def __init__(self):
    self.links = []
    self.joints = []
    self.models_path = os.path.expanduser('~/.gazebo/models/')

  def __repr__(self):
    return 'Model(name=%s,\n links=%s,\n joints=%s\n)' % (self.name, str(self.links), str(self.joints))

  def load_toplevel_sdf(self, sdf_filename):
    tree = ET.parse(sdf_filename)
    sdf = tree.getroot()
    model = sdf.findall('model')[0]
    self.name = model.attrib['name']
    self.load_sdf(sdf_filename)

  def load_sdf(self, sdf_filename, model_prefix = ''):
    tree = ET.parse(sdf_filename)
    sdf = tree.getroot()
    model = sdf.findall('model')[0]
    for link in model.iter('link'):
      self.links.append(Link(link, model_prefix))
    for joint in model.iter('joint'):
      self.joints.append(Joint(joint, model_prefix))
    for include in model.iter('include'):
      included_sdf_filename = include.find('uri').text.replace('model://', self.models_path) + os.path.sep + 'model.sdf'
      name_tag = include.find('name')
      if name_tag != None:
        model_name = name_tag.text
      else:
        model_name = include.find('uri').text.replace('model://', '')
      pose_tag = include.find('pose')
      if pose_tag != None:
        pose_abs = pose_tag.text.replace('\n', ' ').strip()
        for joint in self.joints:
          if joint.child.startswith(model_name):
            parent_pose_abs_tf = self.absolute_position(joint.parent)
            pose_rel_tf = tf_multiply(inverse_matrix(parent_pose_abs_tf), pose2tf(pose_abs))
            pose_rel = tf2pose(pose_rel_tf)
            #print('Setting origin of %s to %s because of pose_abs=%s and parent_pose_abs=%s' % (joint, pose_rel, pose_abs, tf2pose(parent_pose_abs_tf)))
            joint.pose = pose_rel
      if model_prefix:
        model_name = model_prefix + '::' + model_name
      self.load_sdf(included_sdf_filename, model_name)

  def absolute_position(self, link):
    curr_link = link
    curr_pose_tf = identity_matrix()
    had_parent = True
    while had_parent:
      had_parent = False;
      #print('curr_link=%s curr_pose_tf=\n%s' % (curr_link, curr_pose_tf))
      for joint in self.joints:
        if joint.child == curr_link:
          curr_pose_tf = tf_multiply(curr_pose_tf, pose2tf(joint.pose))
          curr_link = joint.parent
          had_parent = True
          break
    return curr_pose_tf


  def save_urdf(self, urdf_filename):
    urdf = ET.Element('robot', {'name': self.name})
    for link in self.links:
      link.toUrdfSubElement(urdf)
    for joint in self.joints:
      joint.toUrdfSubElement(urdf)

    urdf_file = open(urdf_filename, 'w')
    pretty_urdf_string = prettyXML(ET.tostring(urdf))
    urdf_file.write(pretty_urdf_string)




def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('sdf', help='SDF file to convert')
  parser.add_argument('urdf', help='Resulting URDF file to be written')
  args = parser.parse_args()

  model = Model()
  model.load_toplevel_sdf(args.sdf)
  print('Parsed SDF model:\n' + str(model))
  model.save_urdf(args.urdf)


if __name__ == '__main__':
  main()
