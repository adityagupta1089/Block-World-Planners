from PIL import Image, ImageDraw, ImageFont
from moviepy.editor import *
import sys


def parse_state(state):
    table = []
    nxt = [-1] * (n + 1)
    _prob = ([], -1)
    for prop in state:
        x = prop.split(" ")
        if x[0] == "ontable":
            table.append(int(x[1]))
        elif x[0] == "on":
            nxt[int(x[2])] = int(x[1])
        elif x[0] == "hold":
            _prob = (_prob[0], int(x[1]),)
    for x in table:
        stack = [x]
        while nxt[x] != -1:
            x = nxt[x]
            stack.append(x)
        _prob[0].append(stack)
    return _prob


def parse(prob):
    global n
    with open(prob, 'r') as f:
        c = f.readlines()
    c = [x.strip() for x in c]
    states = []
    n = int(c[0])
    for _c in c[1:]:
        states.append(parse_state((_c[1:-1].split(") ("))))
    return states


def draw_frame(prob):
    edge = 70
    gap = 20
    n_w = 2
    n_h = 5
    _w = n_w * edge + (n_w + 1) * gap
    _h = n_h * edge + (n_h + 1) * gap
    im = Image.new("RGB", (_w, _h), "white")
    draw = ImageDraw.Draw(im)
    draw.line((gap, _h - gap / 2, _w - gap, _h - gap / 2), fill="black")
    s = 0
    for stack in prob[0]:
        i = 0
        for block in stack:
            x0 = (s + 1) * gap + s * edge
            y0 = _h - ((i + 1) * gap + i * edge)
            draw.rectangle((x0, y0, x0 + edge, y0 - edge),
                           fill="grey")
            arial = ImageFont.truetype('arial.ttf', 30)
            w, h = arial.getsize(str(block))
            draw.text((x0 + edge / 2 - w / 2, y0 - edge / 2 - h / 2), str(block), fill="black",
                      font=arial)
            i += 1
        s += 1
    if prob[1] != -1:
        x0 = gap
        y0 = gap
        draw.rectangle((x0, y0, x0 + edge, y0 + edge),
                       fill="grey")
        arial = ImageFont.truetype('arial.ttf', 30)
        w, h = arial.getsize(str(prob[1]))
        draw.text((x0 + edge / 2 - w / 2, y0 + edge / 2 - h / 2), str(prob[1]), fill="black",
                  font=arial)
    return im


cnt = 0
for state in parse(sys.argv[1]):
    draw_frame(state).save("out/" + str(cnt)+".png", "png")
    cnt += 1
