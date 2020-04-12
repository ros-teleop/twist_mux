#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# twist_mux: rate_publishers.py
#
# Copyright (c) 2014 PAL Robotics SL.
# All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
#   * Siegfried-A. Gevatter

import sys
import _thread
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class _RatePublisher(Node):

    # How many seconds before the expected time a message may
    # be published.
    _tolerance = 0.01

    def __init__(self, topic, msg_type, context, latch=False):
        super().__init__('rate_publisher_'+topic, context=context)
        self._topic = topic
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(msg_type, topic, qos_profile=latching_qos)
        self._message = None
        self._period = None  # 1 / freq
        self._last_pub = 0

    def pub(self, message, rate=None):
        self._message = message
        self._period = (1. / rate) if rate else None
        self.publish_once()

    def stop(self):
        self._period = None

    def publish_once(self):
        msg = self._message() if callable(self._message) else self._message
        self._publisher.publish(msg)
        self._last_pub = self.get_clock().now()

    def spin_once(self):
        """
        Give _RatePublisher a chance to publish a stored message.

        This method returns the remaining time until the next scheduled
        publication (or -1).
        """
        if not self._period:
            return -1
        elapsed = (self.get_clock().now() - self._last_pub).nanoseconds / 1e9
        if elapsed >= (self._period - self._tolerance):
            self.publish_once()
            return self._period
        return self._period - elapsed


class RatePublishers(object):
    """
    A class for managing several ROS publishers repeating messages
    with different rates.

    The main purpose of this class is for unit testing.
    """

    def __init__(self, context):
        self._publishers = {}
        self._context = context

    def add_topic(self, topic, msg_type):
        """
        Adds a topic for future publication.

        This creates a rospy.Publisher internally. Note that the
        publisher will latch the topic; if that wasn't the case,
        clients might need to sleep before publishing something
        for the first time to give subscribers enough time to
        connect.
        """
        assert topic not in self._publishers
        rate_publisher = _RatePublisher(topic, msg_type, self._context, latch=True)
        self._publishers[topic] = rate_publisher
        return rate_publisher

    def pub(self, topic, message, rate=None):
        """
        Publishes `message' on the given topic.

        If `rate' is not None, the message will be repeated at the
        given rate (expected to be in Hz) until pub() or stop()
        are invoked again.

        Note that `rate' may also be a function, in which case
        it'll be invoked for each publication to obtain the message.
        """
        self._publishers[topic].pub(message, rate)

    def stop(self, topic):
        """
        Stops repeating any message on the given topic.
        """
        self._publishers[topic].stop()

    def spin_once(self):
        """
        Publishes any scheduled messages and returns the amount of
        time until it should be called again.
        """
        # TODO: Create a class that spawns a global thread and provides
        #       createTimer and createWallTimer, just like NodeHandle
        #       does in rospy?
        next_timeout = sys.maxsize
        for topic, pub in self._publishers.items():
            next_timeout = min(pub.spin_once(), next_timeout)
        return next_timeout


# TODO: Make this class more generic (keeping track of timeouts itself)?
class TimeoutManager(object):

    def __init__(self):
        self._shutdown = False
        self._members = []

    def add(self, m):
        self._members.append(m)

    def spin(self):
        while rclpy.ok() and not self._shutdown:
            try:
                for m in self._members:
                    m.spin_once()
                    sleep(0.01)  # FIXME
            except Exception as e:
                print(str(e))
                raise e

    def spin_thread(self):
        print("Spawning thread for TopicTestManager...")
        _thread.start_new_thread(self.spin, ())

    def shutdown(self):
        self._shutdown = True
