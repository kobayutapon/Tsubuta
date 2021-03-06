#!/usr/bin/env python
# -*- coding: utf-8 -*-

from oauth2client.client import SignedJwtAssertionCredentials
import gdata.spreadsheets.client
from xml.etree.ElementTree import *
import os

# import picamera
import time

import wiringpi2
import struct
from time import sleep

import json
from requests_oauthlib import OAuth1Session

import ConfigParser


# for Tsubuta 
# addon raspbian package : fswebcam
#

# addon python plugin
#  gdata oauth2client 
# pip install requests requests_oauthlib


# =========================================
#  Class : Google Docs API(Spread sheet)
# =========================================
# need following packages
# $sudo pip install gdata oauth2client
class GdocSpreadSheet:
	def __init__(self):
		self.client_emails = ""
		self.seckey = ""
		self.sprd_id = ""
		self.cell_tag = ""
		self.sprd_id = ""
		return
		
	def InitConnection(self):
		# 認証に必要な情報
		self.client_email = self.client_emails # 手順2で発行されたメールアドレス
		with open(self.seckey) as f: self.private_key = f.read() # 手順2で発行された秘密鍵
		# 認証情報の作成
		self.scope = ["https://spreadsheets.google.com/feeds"]
		self.credentials = SignedJwtAssertionCredentials(self.client_email, self.private_key, scope=self.scope)

		# スプレッドシート用クライアントの準備
		self.client = gdata.spreadsheets.client.SpreadsheetsClient()

		# OAuth2.0での認証設定
		self.auth_token = gdata.gauth.OAuth2TokenFromCredentials(self.credentials)
		self.auth_token.authorize(self.client)
		
		return
		
	# CellEntryから値を文字列で取得する
	def getCellText( self, cellentry ):
		self.xml_str = cellentry.to_string()
		self.elem = fromstring(self.xml_str)
		self.value = self.elem.findtext(self.cell_tag);
		return self.value
		
	# 指定したセルから文字列を取得する。
	def getCellValue( self, x, y ):
		self.value = self.client.get_cell( self.sprd_id, "od6", x, y )	# セルの値を取得
		return self.getCellText( self.value )


# =========================================
#  Class : tweet
# =========================================
class Tweet:
	def __init__(self):
		self.CONSUMER_KEY      = ''
		self.CONSUMER_SECRET   = ''
		self.ACCESS_KEY      = ''
		self.ACCESS_SECRET   = ''
		self.url_media = "https://upload.twitter.com/1.1/media/upload.json"
		self.url_text = "https://api.twitter.com/1.1/statuses/update.json"
		self.ImageFile = "tmpImage.jpg"

	def tweetMessageWithImage( self, message ):
		self.twitter = OAuth1Session(self.CONSUMER_KEY, self.CONSUMER_SECRET, self.ACCESS_KEY, self.ACCESS_SECRET)

		# 画像投稿
		#print "Start Post Image file.\n"
		self.files = {"media" : open(self.ImageFile, 'rb')}
		self.req_media = self.twitter.post(self.url_media, files = self.files)

		# レスポンスを確認
		#print "Check response.\n"
		if self.req_media.status_code != 200:
			print ("failed to update image: %s", self.req_media.text)
			return False

		# Media ID を取得
		#print "Get Media ID.\n"
		self.media_id = json.loads(self.req_media.text)['media_id']
		print ("Media ID: %d" % self.media_id)

		# Media ID を付加してテキストを投稿
		print "Post Text\n"
		self.params = {'status': message, "media_ids": [self.media_id]}
		#params = {'status': message}
		self.req_media = self.twitter.post(self.url_text, params = self.params)

		# 再びレスポンスを確認
		if self.req_media.status_code != 200:
    			print ("failed to update text message: %s", self.req_text.text)
			return False
		
		return True

	def tweetMessage( self, message ):
		self.twitter = OAuth1Session(self.CONSUMER_KEY, self.CONSUMER_SECRET, self.ACCESS_KEY, self.ACCESS_SECRET)


		# Media ID を付加してテキストを投稿
		print "Post Text\n"
		self.params = {'status': message}
		self.req_media = self.twitter.post(self.url_text, params = self.params)

		# 再びレスポンスを確認
		if self.req_media.status_code != 200:
    			print ("failed to update text message: %s", self.req_text.text)
			return False
		
		return True

# =========================================
#  Class : HDC1000
# =========================================
class HDC1000:
	def __init__(self):
		self.DevAddress = 0x40
		self.wait = (6350.0 + 6500.0 +  500.0)/1000000.0
		self.Temp = -273.1
		self.Hudi = -1

	def getData(self):
		wiringpi2.wiringPiSetup()
		self.i2c = wiringpi2.I2C()
		self.dev = self.i2c.setup(self.DevAddress)
		self.i2c.writeReg16(self.dev,0x02,0x10) #Temp + Hidi 32-bit transfer mode, LSB-MSB inverted, why?
		self.i2c.writeReg8(self.dev,0x00,0x00) #start conversion.
		sleep( self.wait ) #wait for conversion.
		#LSB-MSB inverted, again...
		self.temp = ((struct.unpack('4B', os.read(self.dev,4)))[0] << 8 | (struct.unpack('4B', os.read(self.dev,4)))[1])
		self.hudi = ((struct.unpack('4B', os.read(self.dev,4)))[2] << 8 | (struct.unpack('4B', os.read(self.dev,4)))[3])
		os.close(self.dev) #Don't leave the door open.

		self.Hudi = ( self.hudi / 65535.0 ) * 100
		self.Temp = ( self.temp  / 65535.0) * 165 - 40

		return

	def getHumidity(self):
		return self.Hudi

	def getTemperature(self):
		return self.Temp

# ===============================================

# main function

a1 = GdocSpreadSheet()
tweet = Tweet() 
sensor = HDC1000()


# load setting from Google docs.
a1.InitConnection()
account = a1.getCellValue(2,1)
key1 = a1.getCellValue(2,2)
key2 = a1.getCellValue(2,3)
hashtag1 = a1.getCellValue(2,4)
hashtag2 = a1.getCellValue(2,5)
comment = a1.getCellValue(2,6)
tweet_temp = a1.getCellValue(2,7)
tweet_hurmidity = a1.getCellValue(2,8)
tweet_camera = a1.getCellValue(2,9)
interval = a1.getCellValue(2,10)

print account
print key1
print key2
print hashtag1
print hashtag2
print comment
print tweet_temp
print tweet_hurmidity
print tweet_camera
print interval

Temperature = -273.1
Hurmidity = -1

# Get Sensor Data
if tweet_temp == "1" or tweet_hurmidity == "1" :
	sensor = HDC1000()
	sensor.getData()
	Temperature = sensor.getTemperature()
	Hurmidity = sensor.getHumidity()
#	Temperature = 10.0 # for Debug
#	Hurmidity = 50 # for Debug
	print "get Sensor"

# Camera
if tweet_camera == "1" :
	extcode = os.system('raspistill -w 1280 -h 800 -o tmpImage.jpg')
	if extcode != 0 :
		print "Error: Failed to capture image."
		tweet_camera = "0"
	else:
		print "get Image"

elif tweet_camera == "2" :
	extcode = os.system('fswebcam -r 1280x800 tmpImage.jpg --jpeg 95 --title "photo by tsubuta"')
	if extcode != 0 :
		print "Error: Failed to capture image."
		tweet_camera = "0"
	else:
		print "get Image"


message = comment + "\r\n"
if tweet_temp == "1" :
	message += u"温度：" + "{0:.2f}".format(Temperature) +u"℃\r\n"

if tweet_hurmidity == "1" :
	message += u"湿度：" + u"{0:.2f}".format(Hurmidity) + u"%\r\n"

if hashtag1 != "" :
	message += u"#" + hashtag1

if hashtag2 != "" :
	message += u"#" + hashtag2

print message

if tweet_camera == "1" :
	tweet.tweetMessageWithImage(message)
else:
	tweet.tweetMessage(message)

# ここでGoogle Docsのスプレッドシートとかにデータをあげるのもあり。

