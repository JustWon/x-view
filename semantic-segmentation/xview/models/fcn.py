import tensorflow as tf
from tensorflow.python.layers.layers import conv2d, max_pooling2d, dropout

from .base_model import BaseModel
from .custom_layers import deconv2d, log_softmax, softmax


class FCN(BaseModel):
    """FCN implementation following DA-RNN architecture and using tf.layers."""

    def __init__(self, output_dir=None, **config):
        BaseModel.__init__(self, 'FCN', output_dir=output_dir, **config)

    def _build_graph(self):
        # the input
        # rgb channel
        self.X_rgb = tf.placeholder(tf.float32, shape=[None, None, None, 3])
        # depth channel
        self.X_d = tf.placeholder(tf.float32, shape=[None, None, None, 3])
        # ground truth label
        self.Y_in = tf.placeholder(tf.float32, shape=[None, None, None,
                                                      self.config['num_classes']])
        # dropout keep probability
        self.dropout_rate = tf.placeholder(tf.float32)
        # queue
        q = tf.FIFOQueue(100, [tf.float32, tf.float32, tf.float32, tf.float32])
        self.enqueue_op = q.enqueue([self.X_rgb, self.X_d, self.Y_in, self.dropout_rate])
        rgb, depth, training_labels, dropout_rate = q.dequeue()
        # The queue output does not have a defined shape, so we have to define it here.
        rgb.set_shape([None, None, None, 3])
        depth.set_shape([None, None, None, 3])
        self.close_queue_op = q.close(cancel_pending_enqueues=True)

        # standard parameters for convolutions
        params = {'activation': tf.nn.relu, 'padding': 'same'}

        def vgg16(inputs, prefix):
            """VGG16 image encoder."""
            # common parameters
            net = conv2d(inputs, 64, [3, 3], name='{}_conv1_1'.format(prefix), **params)
            net = conv2d(net, 64, [3, 3], name='{}_conv1_2'.format(prefix), **params)
            net = max_pooling2d(net, [2, 2], [2, 2], name='{}_pool1'.format(prefix))
            net = conv2d(net, 128, [3, 3], name='{}_conv2_1'.format(prefix), **params)
            net = conv2d(net, 128, [3, 3], name='{}_conv2_2'.format(prefix), **params)
            net = max_pooling2d(net, [2, 2], [2, 2], name='{}_pool2'.format(prefix))
            net = conv2d(net, 256, [3, 3], name='{}_conv3_1'.format(prefix), **params)
            net = conv2d(net, 256, [3, 3], name='{}_conv3_2'.format(prefix), **params)
            net = conv2d(net, 256, [3, 3], name='{}_conv3_3'.format(prefix), **params)
            net = max_pooling2d(net, [2, 2], [2, 2], name='{}_pool3'.format(prefix))
            net = conv2d(net, 512, [3, 3], name='{}_conv4_1'.format(prefix), **params)
            net = conv2d(net, 512, [3, 3], name='{}_conv4_2'.format(prefix), **params)
            conv4 = conv2d(net, 512, [3, 3], name='{}_conv4_3'.format(prefix), **params)
            net = max_pooling2d(conv4, [2, 2], [2, 2], name='{}_pool4'.format(prefix))
            net = conv2d(net, 512, [3, 3], name='{}_conv5_1'.format(prefix), **params)
            net = conv2d(net, 512, [3, 3], name='{}_conv5_2'.format(prefix), **params)
            conv5 = conv2d(net, 512, [3, 3], name='{}_conv5_3'.format(prefix), **params)
            return conv5, conv4

        # each modality encoded by vgg 16
        rgb_conv5, rgb_conv4 = vgg16(rgb, 'rgb')
        depth_conv5, depth_conv4 = vgg16(depth, 'depth')

        # At the output of stages conv4_3 and conv5_3, stack the 2 modalities together.
        conv4 = tf.concat([rgb_conv4, depth_conv4], 3, name='concat_conv4')
        conv4 = conv2d(conv4, self.config['num_units'], [1, 1], name='score_conv4',
                       **params)

        # Output of conv5_3 is upsampled to match the dimensions.
        conv5 = tf.concat([rgb_conv5, depth_conv5], 3, name='concat_conv5')
        conv5 = conv2d(conv5, self.config['num_units'], [1, 1], name='score_conv5',
                       **params)
        conv5 = deconv2d(conv5, self.config['num_units'], [4, 4], strides=[2, 2],
                         name='upscore_conv5', trainable=False, **params)

        fused = tf.add_n([conv4, conv5], name='add_score')
        fused = dropout(fused, rate=dropout_rate, name='dropout')

        # Upsample the fused features to the output classification size
        fused = deconv2d(fused, self.config['num_units'], [16, 16], strides=[8, 8],
                         name='upscore', trainable=False, **params)
        score = conv2d(fused, self.config['num_classes'], [1, 1], name='score',
                       **params)

        # Normalize probabilities to 1.
        prob = log_softmax(score, self.config['num_classes'], name='prob')

        # For classification, we want to output the argmax of the probability vector.
        label = softmax(score, self.config['num_classes'], name='prob_normalized')
        label = tf.argmax(label, 3, name='label_2d')

        # Now we expose the different used outputs as class properties.
        self.class_probabilities = prob
        self.prediction = label
        self.Y = training_labels

    def _enqueue_batch(self, batch, sess):
        with self.graph.as_default():
            feed_dict = {self.X_rgb: batch['rgb'], self.X_d: batch['depth'],
                         self.Y_in: batch['labels'],
                         self.dropout_rate: batch['dropout_rate']}
            sess.run(self.enqueue_op, feed_dict=feed_dict)
