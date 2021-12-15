import curses

screen = curses.initscr()

screen.addstr("Get char")
screen.refresh()

c = screen.getch()
curses.endwin()

print(c)