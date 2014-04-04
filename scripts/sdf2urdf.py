#!/usr/bin/env python

from __future__ import print_function

import sys
import itertools
import os
import argparse
import xml.etree.ElementTree as ET
import xml.dom.minidom
from tf.transformations import *

def rounded(val):
  return int(round(val,6) * 1e5) / 1.0e5

def round_all(lst):
  return [rounded(item) for item in lst]

def is_same_transform_rough(matrix0, matrix1):
  matrix0 = numpy.array(matrix0, dtype=numpy.float64, copy=True)
  matrix0 /= matrix0[3, 3]
  matrix1 = numpy.array(matrix1, dtype=numpy.float64, copy=True)
  matrix1 /= matrix1[3, 3]
  return numpy.allclose(matrix0, matrix1, 1e-04, 1e-04)

def str2float_list(s):
  return [float(i) for i in s.split()]

def prettyXML(uglyXML):
  return xml.dom.minidom.parseString(uglyXML).toprettyxml(indent='  ')

def pose2origin(pose):
  xyz = ' '.join(pose.split()[:3])
  rpy = ' '.join(pose.split()[3:])
  return xyz, rpy

def pose2tf(pose):
  pose_float = str2float_list(pose)
  translate = pose_float[:3]
  angles = pose_float[3:]
  tf = compose_matrix(None, None, angles, translate)
  return tf

def tf2pose(tf):
  scale, shear, angles, trans, persp = decompose_matrix(tf)

  # "normalize" (for "common" angles) to euler angles as close to zero as possible and prefer positive to negative values
  angles = round_all(angles)
  orig_tf = compose_matrix(None, None, angles)
  for i in numpy.linspace(1, -1, 5):
    for j in numpy.linspace(1, -1, 5):
      for k in numpy.linspace(1, -1, 5):
          delta = [val * 3.14159 for val in i, j, k]
          angles_mod = round_all(sum(x) for x in zip(angles, delta))
          if sum(abs(v) for v in angles) <= sum(abs(vm) for vm in angles_mod) \
             or any(rounded(abs(vm)) > 3.14159 for vm in angles_mod):
            continue
          mod_tf = compose_matrix(None, None, angles_mod)
          if is_same_transform_rough(orig_tf, mod_tf):
            #print('Using angles %s instead of %s' % (angles_mod, angles))
            angles = angles_mod

  return ' '.join(str(rounded(i)) for i in itertools.chain(trans, angles))

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

def tf_strvector_multiply(tf, vector):
  vector_tf = translation_matrix(str2float_list(vector))
  result_tf = translation_from_matrix(tf_multiply(tf, vector_tf))
  return ' '.join(str(rounded(i)) for i in result_tf)

def abs2rel(tf_base, tf_abs):
  return tf_multiply(inverse_matrix(tf_base), tf_abs)

def extract_rotation(tf):
  scale, shear, angles, trans, persp = decompose_matrix(tf)
  return compose_matrix(None, None, angles)


class Entity(object):
  def __init__(self):
    self.sdf_pose = '0 0 0 0 0 0'
    self.urdf_pose = '0 0 0 0 0 0'

  def set_urdf_pose(self, tf):
    self.urdf_pose = tf2pose(tf)


class Link(Entity):
  def __init__(self, link_tag, prefix = '', pose = '0 0 0 0 0 0'):
    super(Link, self).__init__()
    self.name = link_tag.attrib['name']
    if prefix:
      self.name = prefix + '_' + self.name
    self.sdf_pose = pose
    self.inertial = {}
    self.collision = {}
    self.visual = {}
    self.joints = None

    pose_tag = link_tag.find('pose')
    if pose_tag != None:
      self.sdf_pose = pose_multiply(self.sdf_pose, pose_tag.text.replace('\n', ' ').strip())

    inertial = link_tag.find('inertial')
    if inertial != None:
      pose_tag = inertial.find('pose')
      if pose_tag != None:
        self.inertial['sdf_pose'] = pose_tag.text.replace('\n', ' ').strip()
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
        pose_tag = elem_tag.find('pose')
        if pose_tag != None:
          getattr(self, elem)['sdf_pose'] = pose_tag.text
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
      if 'urdf_pose' in self.inertial:
        xyz, rpy = pose2origin(self.inertial['urdf_pose'])
        origin_tag = ET.SubElement(inertial_tag, 'origin', {'rpy': rpy, 'xyz': xyz})
      inertia_tag = ET.SubElement(inertial_tag, 'inertia')
      for coord in 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz':
        if 'inertia' in self.inertial:
          inertia = self.inertial['inertia']
        else:
          inertia = {}
        inertia_tag.attrib[coord] = inertia.get(coord, '0')

  def __repr__(self):
    return 'Link(name=%s, sdf_pose=%s, urdf_pose=%s, inertial=%s, collision=%s, visual=%s, joints=%s)' % (self.name, self.sdf_pose, self.urdf_pose, self.inertial, self.collision, self.visual, [joint.name for joint in self.joints])




class Joint(Entity):
  def __init__(self, joint_tag, prefix = ''):
    super(Joint, self).__init__()
    self.name = joint_tag.attrib['name']
    self.joint_type = joint_tag.attrib['type']
    self.child = joint_tag.find('child').text.replace('::', '_')
    self.parent = joint_tag.find('parent').text.replace('::', '_')
    if prefix:
      self.name = prefix + '_' + self.name
      self.child = prefix + '_' + self.child
      self.parent = prefix + '_' + self.parent
    self.sdf_pose = '0 0 0 0 0 0'
    self.urdf_pose = '0 0 0 0 0 0'
    self.axis = {}

    pose_tag = joint_tag.find('pose')
    if pose_tag != None:
      self.sdf_pose = pose_tag.text.replace('\n', ' ').strip()
    axis_tag = joint_tag.find('axis')
    xyz_tag = axis_tag.find('xyz')
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

    xyz, rpy = pose2origin(self.urdf_pose)
    origin_tag = ET.SubElement(joint_tag, 'origin', {'rpy': rpy, 'xyz': xyz})

    axis_tag = ET.SubElement(joint_tag, 'axis')
    axis_tag.attrib['xyz'] = self.axis['xyz']
    if 'limit' in self.axis and joint_tag.attrib['type'] != 'fixed':
      limit_tag = ET.SubElement(joint_tag, 'limit')
      for attrib in 'lower', 'upper', 'effort', 'velocity':
        if attrib in self.axis['limit']:
          limit_tag.attrib[attrib] = self.axis['limit'][attrib]

  def __repr__(self):
    return 'Joint(name=%s, sdf_pose=%s, urdf_pose=%s, type=%s, child=%s, parent=%s, axis=%s)' % (self.name, self.sdf_pose, self.urdf_pose, self.joint_type, self.child, self.parent, str(self.axis))

  def rotateUrdfAxis(self, tf):
    print('before=%s' % self.axis['xyz'])
    print('tf:\n%s' % tf)
    rotation_tf = extract_rotation(tf)
    self.axis['xyz'] = tf_strvector_multiply(rotation_tf, self.axis['xyz'])
    print('after=%s' % self.axis['xyz'])




class Model:
  def __init__(self):
    self.links = []
    self.joints = []
    self.models_path = os.path.expanduser('~/.gazebo/models/')
    self.root_link = ''

  def __repr__(self):
    return 'Model(name=%s,\n root_link=%s,\n links:\n    %s,\n joints:\n    %s\n)' % (self.name, self.root_link.name, '\n    '.join(str(link) for link in self.links), '\n    '.join(str(joint) for joint in self.joints))

  def load_toplevel_sdf(self, sdf_filename):
    tree = ET.parse(sdf_filename)
    sdf = tree.getroot()
    self.sdf_version = float(sdf.attrib['version'])
    model = sdf.findall('model')[0]
    self.name = model.attrib['name']
    self.load_sdf(sdf_filename)
    self.root_link = self.find_root_link()
    self.build_model_tree(self.root_link)
    self.set_urdf_pose(self.root_link, identity_matrix())

  def load_sdf(self, sdf_filename, model_prefix = '', pose = '0 0 0 0 0 0'):
    tree = ET.parse(sdf_filename)
    sdf = tree.getroot()
    model = sdf.findall('model')[0]
    for link in model.iter('link'):
      self.links.append(Link(link, model_prefix, pose))
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
      # TODO modify joint axis?
      if pose_tag != None:
        include_pose = pose_multiply(pose, pose_tag.text.replace('\n', ' ').strip())
      else:
        include_pose = pose
      if model_prefix:
        model_name = model_prefix + '_' + model_name
      self.load_sdf(included_sdf_filename, model_name, include_pose)

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

  def find_root_link(self):
    curr_link = self.links[0]
    had_parent = True
    while had_parent:
      had_parent = False;
      for joint in self.joints:
        if joint.child == curr_link:
          curr_link = joint.parent
          had_parent = True
          break
    return curr_link

  def get_link(self, link_name):
    for link in self.links:
      if link.name == link_name:
        return link

  def build_model_tree(self, link):
    link.joints = self.find_joints(link)
    for child in [joint.child for joint in link.joints]:
      self.build_model_tree(self.get_link(child))

  def set_urdf_pose(self, link, joint_abs_tf):
    print('link=%s joint_abs_tf=%s' % (link.name, tf2pose(joint_abs_tf)))
    link.set_urdf_pose(abs2rel(pose2tf(link.sdf_pose), joint_abs_tf))
    for joint in link.joints:
      joint_child = self.get_link(joint.child)
      joint_rel_tf = abs2rel(joint_abs_tf, pose2tf(joint_child.sdf_pose))
      print('joint=%s joint_child=%s joint_child.sdf_pose=%s -> joint_rel_tf=%s' % (joint.name, joint_child.name, joint_child.sdf_pose, tf2pose(joint_rel_tf)))
      joint.set_urdf_pose(joint_rel_tf)
      # SDF 1.4 axis is specified in model frame, but urdf in joint=child frame
      new_abs_child_tf = tf_multiply(joint_abs_tf, joint_rel_tf)
      if self.sdf_version < 1.5:
        joint.rotateUrdfAxis(new_abs_child_tf)
      else:
        print('Untested SDF version!')
        sys.exit(1)
      self.set_urdf_pose(joint_child, new_abs_child_tf)


  def find_joints(self, link):
    return [joint for joint in self.joints if joint.parent == link.name]

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

  for test_pose in ['0 0 0 0 0 0', '1 0 0 0 0 0', '1 1 0 0 0 0', '-1 0 0 0 0 0', '0 0 0 1 0 0', '0 0 0 0 3.14159 0', '0 0 0 0 1.5708 1.5708', '0 0 0 1 1 0', '0 5 0 -1 0 0', '1 0 1 0 1 1', '0 0 0.5 0 -1.5708 0']:
    test_res = tf2pose(pose2tf(test_pose))
    test_res_float = str2float_list(test_res)
    test_pose_float = str2float_list(test_pose)
    if not numpy.allclose(test_res_float, test_pose_float):
      print('test_pose failed: input=%s output=%s' % (test_pose, test_res))
      #sys.exit(1)

  model = Model()
  model.load_toplevel_sdf(args.sdf)
  print('Parsed SDF model:\n' + str(model))
  model.save_urdf(args.urdf)


if __name__ == '__main__':
  main()
