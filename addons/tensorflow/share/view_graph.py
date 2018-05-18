#!/usr/bin/python

from __future__ import print_function
import tensorflow as tf
import sys, os

from tensorflow.python.platform import gfile

if len(sys.argv) < 2:
    print("Usage:")
    print("  viewgraph.py <model pb>")
    sys.exit(1)

log_dir = os.path.expanduser('~/tmp')

with tf.Session() as sess:
    with gfile.FastGFile(sys.argv[1], 'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
        g_in = tf.import_graph_def(graph_def)
train_writer = tf.summary.FileWriter(log_dir)
train_writer.add_graph(sess.graph)

print("Make sure to run tensorboard --logdir=%s" % log_dir)
