import pyttsx3 as pyttsx

##  SOUND SETUP
voice = pyttsx.init()
voice.setProperty('volume',1)
voices = voice.getProperty('voices')
for opt in voices:
    if(opt.gender == 'female'):
        voice.setProperty('voice', opt.id[1])
        voice.say(opt.name)
        voice.runAndWait()