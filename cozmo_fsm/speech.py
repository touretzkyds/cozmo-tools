try:
  import speech_recognition as sr
except: paass      

from threading import Thread

from .evbase import Event
from .events import SpeechEvent

class Thesaurus():
    def __init__(self):
        self.entries = dict()
        self.add_homophones('cozmo', \
                            ["cozimo","cosimo","cosmo", \
                             "kozmo","cosmos","cozmos"])

    def add_homophones(self,word,homophones):
        if not isinstance(homophones,list):
            homophones = [homophones]
        for h in homophones:
            self.entries[h] = word

    def lookup(self,word):
        return self.entries.get(word,word)

class SpeechListener():
  def __init__(self,robot,thesaurus):
    self.robot = robot
    self.thesaurus = thesaurus

  def speech_listener(self):
    self.rec = sr.Recognizer()
    with sr.Microphone() as source:
      while True:
          audio = self.rec.listen(source)
          try:
              utterance = self.rec.recognize_google(audio).lower()
              words = [self.thesaurus.lookup(w) for w in utterance.split(" ")]
              string = " ".join(words)
              print("Heard: '%s'" % string)
              evt = SpeechEvent(string,words)
              self.robot.erouter.post(evt)
          except sr.UnknownValueError:
              pass
          except sr.RequestError as e:
              print("Could not request results form google speech recognition service; {0}".format(e)) 

  def start(self):
    self.thread = Thread(target=self.speech_listener)
    self.thread.daemon = True #ending fg program will kill bg program
    self.thread.start()
