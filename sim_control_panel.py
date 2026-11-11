#!/usr/bin/env python3
#
# Copyright 2026 Sanaullah
# SPDX-License-Identifier: Apache-2.0
#
# Mission-control GUI for FFW-SG2 navigation in simulation.
#
# One window that:
#   - starts/stops the whole stack via run_nav_sim.sh (SLAM or saved-map mode)
#   - lets you pick WHICH saved map to load for saved-map nav
#   - saves the map (runs map_saver_cli)
#   - deletes a saved map (.yaml + its image file)
#   - drives the swerve base (publishes Twist to /cmd_vel)
#
# Run this DIRECTLY inside the ai_worker container:
#     python3 sim_control_panel.py
#
# Needs Tkinter:  apt-get update && apt-get install -y python3-tk

import os
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PUBLISH_HZ = 20
CMD_VEL_TOPIC = '/cmd_vel'
SCRIPT_DIR = '/root/ros2_ws/src/ai_worker'
RUN_SCRIPT = 'run_nav_sim.sh'
MAPS_DIR = '/root/ros2_ws/src/ai_worker/ffw_navigation/maps'
DEFAULT_MAP_PATH = os.path.join(MAPS_DIR, 'map')
LOGDIR = '/tmp/nav_sim_logs'
NO_MAPS = '(no maps - save one first)'


class TeleopNode(Node):
    def __init__(self):
        super().__init__('sim_control_panel')
        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.vx = self.vy = self.wz = 0.0

    def publish(self):
        t = Twist()
        t.linear.x = float(self.vx)
        t.linear.y = float(self.vy)
        t.angular.z = float(self.wz)
        self.pub.publish(t)

    def zero(self):
        self.vx = self.vy = self.wz = 0.0


class Panel:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.pressed = set()
        self._release_jobs = {}
        self._zero_burst = 0

        os.makedirs(LOGDIR, exist_ok=True)

        self.proc = None
        self.busy = False
        self.mode = 'stopped'
        self.loaded_map = None          # yaml path used by the running saved-map stack
        self.status_msg = 'stack stopped - press Start mapping'
        self.status_color = '#9399b2'
        self.save_msg = ''
        self.save_color = '#9399b2'
        self.saving = False

        root.title('FFW-SG2 Sim Control')
        root.configure(bg='#1e1e2e', padx=16, pady=14)
        root.minsize(480, 0)
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        tk.Label(root, text='FFW-SG2 Simulation Control', font=('Helvetica', 16, 'bold'),
                 fg='#cdd6f4', bg='#1e1e2e').pack()

        # ---- mission control: start/stop ----
        mc = tk.Frame(root, bg='#1e1e2e')
        mc.pack(pady=(12, 4), fill='x')
        self.btn_slam = tk.Button(mc, text='Start mapping\n(SLAM)', width=15, height=2,
                                  font=('Helvetica', 10, 'bold'), relief='flat', bd=0,
                                  fg='#11111b', bg='#89b4fa', activebackground='#74a8f0',
                                  command=lambda: self.launch('slam'))
        self.btn_slam.grid(row=0, column=0, padx=3)
        self.btn_map = tk.Button(mc, text='Start saved-map\nnav', width=15, height=2,
                                 font=('Helvetica', 10, 'bold'), relief='flat', bd=0,
                                 fg='#11111b', bg='#a6e3a1', activebackground='#94d3a2',
                                 command=lambda: self.launch('map'))
        self.btn_map.grid(row=0, column=1, padx=3)
        self.btn_stop = tk.Button(mc, text='Stop all', width=10, height=2,
                                  font=('Helvetica', 10, 'bold'), relief='flat', bd=0,
                                  fg='#11111b', bg='#f38ba8', activebackground='#eba0ac',
                                  command=self.stop_all)
        self.btn_stop.grid(row=0, column=2, padx=3)

        # ---- map picker (feeds "Start saved-map nav") ----
        mp = tk.Frame(root, bg='#1e1e2e')
        mp.pack(pady=(6, 2), fill='x')
        tk.Label(mp, text='Load map:', font=('Helvetica', 10),
                 fg='#cdd6f4', bg='#1e1e2e').pack(side='left')
        self.map_choice = tk.StringVar(value=NO_MAPS)
        self.map_dropdown = tk.OptionMenu(mp, self.map_choice, NO_MAPS)
        self.map_dropdown.config(font=('Helvetica', 10), bg='#313244', fg='#cdd6f4',
                                 activebackground='#45475a', activeforeground='#cdd6f4',
                                 highlightthickness=0, relief='flat', width=22)
        self.map_dropdown['menu'].config(bg='#313244', fg='#cdd6f4')
        self.map_dropdown.pack(side='left', padx=6)
        tk.Button(mp, text='\u21bb', font=('Helvetica', 11, 'bold'), relief='flat', bd=0,
                  fg='#cdd6f4', bg='#45475a', activebackground='#585b70',
                  command=self.refresh_maps).pack(side='left')
        self.btn_del = tk.Button(mp, text='\U0001F5D1 Delete', font=('Helvetica', 10, 'bold'),
                                 relief='flat', bd=0, fg='#11111b', bg='#f38ba8',
                                 activebackground='#eba0ac', command=self.delete_map)
        self.btn_del.pack(side='left', padx=(6, 0))

        self.status = tk.Label(root, text='', font=('Helvetica', 9),
                               fg='#9399b2', bg='#1e1e2e', wraplength=440)
        self.status.pack(pady=(4, 0))

        # ---- save map ----
        tk.Frame(root, height=1, bg='#45475a').pack(fill='x', pady=(10, 8))
        sm = tk.Frame(root, bg='#1e1e2e')
        sm.pack(fill='x')
        tk.Label(sm, text='Save to:', font=('Helvetica', 9),
                 fg='#cdd6f4', bg='#1e1e2e').pack(side='left')
        self.map_path = tk.Entry(sm, font=('Courier', 9), width=28, bg='#313244',
                                 fg='#cdd6f4', insertbackground='#cdd6f4', relief='flat')
        self.map_path.insert(0, DEFAULT_MAP_PATH)
        self.map_path.pack(side='left', padx=6, fill='x', expand=True)
        self.btn_save = tk.Button(sm, text='Save Map', font=('Helvetica', 10, 'bold'),
                                  relief='flat', bd=0, fg='#11111b', bg='#f9e2af',
                                  activebackground='#f5d889', command=self.save_map)
        self.btn_save.pack(side='left')
        self.save_status = tk.Label(root, text='save while mapping; change the name for a new map',
                                    font=('Helvetica', 9), fg='#9399b2', bg='#1e1e2e')
        self.save_status.pack(pady=(4, 0))

        # ---- manual drive ----
        tk.Frame(root, height=1, bg='#45475a').pack(fill='x', pady=(10, 8))
        tk.Label(root, text='Manual drive  (idle = silent, won\u2019t fight Nav2)',
                 font=('Helvetica', 9), fg='#cdd6f4', bg='#1e1e2e').pack()
        pad = tk.Frame(root, bg='#1e1e2e')
        pad.pack(pady=(6, 0))
        layout = [
            ('\u27f2\nTurn L', 0, 0, self.turn_left),
            ('\u2191\nForward', 0, 1, self.forward),
            ('\u27f3\nTurn R', 0, 2, self.turn_right),
            ('\u2190\nStrafe L', 1, 0, self.strafe_left),
            ('\u25a0\nSTOP', 1, 1, None),
            ('\u2192\nStrafe R', 1, 2, self.strafe_right),
            ('\u2193\nBack', 2, 1, self.backward),
        ]
        for text, r, c, cb in layout:
            stop_btn = cb is None
            b = tk.Button(pad, text=text, width=9, height=2,
                          font=('Helvetica', 10, 'bold'), relief='flat', bd=0,
                          fg='#11111b' if stop_btn else '#cdd6f4',
                          bg='#f38ba8' if stop_btn else '#45475a',
                          activebackground='#eba0ac' if stop_btn else '#585b70')
            b.grid(row=r, column=c, padx=4, pady=4)
            if stop_btn:
                b.bind('<ButtonPress-1>', lambda e: self.stop())
            else:
                b.bind('<ButtonPress-1>', lambda e, f=cb: f())
                b.bind('<ButtonRelease-1>', lambda e: self.stop())

        sf = tk.Frame(root, bg='#1e1e2e')
        sf.pack(pady=(8, 0))
        self.lin = tk.DoubleVar(value=0.4)
        self.ang = tk.DoubleVar(value=0.8)
        self._slider(sf, 'Linear (m/s)', self.lin, 0.0, 1.0, 0)
        self._slider(sf, 'Angular (rad/s)', self.ang, 0.0, 2.0, 1)
        self.drive_status = tk.Label(root, text='', font=('Courier', 10),
                                     fg='#a6e3a1', bg='#1e1e2e')
        self.drive_status.pack(pady=(6, 0))
        tk.Label(root, text='Keys: W/S move \u2022 A/D turn \u2022 Q/E strafe \u2022 Space stop',
                 font=('Helvetica', 8), fg='#9399b2', bg='#1e1e2e').pack()

        root.bind('<KeyPress>', self.on_key_press)
        root.bind('<KeyRelease>', self.on_key_release)
        root.focus_set()
        self.refresh_maps()
        self.tick()

    def _slider(self, parent, label, var, lo, hi, col):
        f = tk.Frame(parent, bg='#1e1e2e')
        f.grid(row=0, column=col, padx=8)
        tk.Label(f, text=label, font=('Helvetica', 9), fg='#cdd6f4', bg='#1e1e2e').pack()
        tk.Scale(f, variable=var, from_=lo, to=hi, resolution=0.05, orient='horizontal',
                 length=150, bg='#1e1e2e', fg='#cdd6f4', troughcolor='#45475a',
                 highlightthickness=0).pack()

    # ---------------- map list ----------------
    def refresh_maps(self):
        try:
            files = sorted(f for f in os.listdir(MAPS_DIR) if f.endswith('.yaml'))
        except OSError:
            files = []
        menu = self.map_dropdown['menu']
        menu.delete(0, 'end')
        if not files:
            self.map_choice.set(NO_MAPS)
            menu.add_command(label=NO_MAPS, command=tk._setit(self.map_choice, NO_MAPS))
            return
        for f in files:
            menu.add_command(label=f, command=tk._setit(self.map_choice, f))
        if self.map_choice.get() not in files:
            self.map_choice.set('map.yaml' if 'map.yaml' in files else files[0])

    # ---------------- delete map ----------------
    @staticmethod
    def _map_image_path(yaml_path):
        """Return the image file referenced by a map yaml (image: ...), or None."""
        try:
            with open(yaml_path) as f:
                for line in f:
                    s = line.strip()
                    if s.startswith('image:'):
                        img = s.split(':', 1)[1].strip().strip('\'"')
                        if not img:
                            return None
                        if not os.path.isabs(img):
                            img = os.path.join(os.path.dirname(yaml_path), img)
                        return img
        except OSError:
            pass
        # fallback: same stem with a common map image extension
        stem = os.path.splitext(yaml_path)[0]
        for ext in ('.pgm', '.png'):
            if os.path.isfile(stem + ext):
                return stem + ext
        return None

    def delete_map(self):
        sel = self.map_choice.get()
        if not sel or sel == NO_MAPS:
            self.status_msg, self.status_color = 'no map selected to delete', '#f38ba8'
            return
        yaml_path = os.path.join(MAPS_DIR, sel)
        if not os.path.isfile(yaml_path):
            self.status_msg, self.status_color = f'map not found: {sel}', '#f38ba8'
            self.refresh_maps()
            return
        running = self.proc is not None and self.proc.poll() is None
        if running and self.mode == 'map' and self.loaded_map == yaml_path:
            self.status_msg = f'{sel} is loaded by the running nav stack - press Stop all first'
            self.status_color = '#f38ba8'
            return

        img = self._map_image_path(yaml_path)
        targets = [yaml_path] + ([img] if img and os.path.isfile(img) else [])
        names = '\n'.join(os.path.basename(t) for t in targets)
        if not messagebox.askyesno('Delete map',
                                   f'Permanently delete these files?\n\n{names}\n\n'
                                   'This cannot be undone.', parent=self.root):
            return

        failed = []
        for t in targets:
            try:
                os.remove(t)
            except OSError as e:
                failed.append(f'{os.path.basename(t)} ({e.strerror})')
        if failed:
            self.status_msg = 'delete failed: ' + ', '.join(failed)
            self.status_color = '#f38ba8'
        else:
            self.status_msg = f'deleted \u2713  {sel}'
            self.status_color = '#a6e3a1'
        self.refresh_maps()

    # ---------------- orchestrator control ----------------
    def launch(self, mode):
        if self.busy:
            return
        map_yaml = None
        if mode == 'map':
            sel = self.map_choice.get()
            if not sel or sel == NO_MAPS:
                self.status_msg = 'no map selected - save one, then press \u21bb refresh'
                self.status_color = '#f38ba8'
                return
            map_yaml = os.path.join(MAPS_DIR, sel)
            if not os.path.isfile(map_yaml):
                self.status_msg = f'map file not found: {map_yaml}'
                self.status_color = '#f38ba8'
                return
        self.busy = True
        label = 'SLAM' if mode == 'slam' else f'saved map ({self.map_choice.get()})'
        self.status_msg = f'starting {label} (stopping any running stack first)\u2026'
        self.status_color = '#f9e2af'
        threading.Thread(target=self._launch_worker, args=(mode, map_yaml), daemon=True).start()

    def _launch_worker(self, mode, map_yaml):
        try:
            self._stop_proc()
            script = os.path.join(SCRIPT_DIR, RUN_SCRIPT)
            if not os.path.isfile(script):
                self.status_msg = f'cannot find {script}'
                self.status_color = '#f38ba8'
                return
            env = os.environ.copy()
            if map_yaml:
                env['MAP_YAML'] = map_yaml
            log = open(os.path.join(LOGDIR, 'panel_orchestrator.log'), 'ab')
            self.proc = subprocess.Popen(
                ['bash', RUN_SCRIPT, mode], cwd=SCRIPT_DIR,
                stdout=log, stderr=log, start_new_session=True, env=env)
            self.mode = mode
            self.loaded_map = map_yaml
            if mode == 'slam':
                self.status_msg = 'mapping running - drive around, watch RViz, then Save Map'
            else:
                self.status_msg = (f'saved-map nav running ({os.path.basename(map_yaml)}) - '
                                   'set 2D Pose Estimate, then 2D Goal Pose')
            self.status_color = '#a6e3a1'
        except Exception as e:
            self.status_msg = f'launch error: {e}'
            self.status_color = '#f38ba8'
        finally:
            self.busy = False

    def stop_all(self):
        if self.busy:
            return
        self.busy = True
        self.status_msg = 'stopping the stack\u2026'
        self.status_color = '#f9e2af'
        threading.Thread(target=self._stop_worker, daemon=True).start()

    def _stop_worker(self):
        try:
            self._stop_proc()
            self.mode = 'stopped'
            self.status_msg = 'stack stopped'
            self.status_color = '#9399b2'
        finally:
            self.busy = False

    def _stop_proc(self):
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
            for _ in range(50):
                if self.proc.poll() is not None:
                    break
                time.sleep(0.2)
            if self.proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        self.proc = None
        self.loaded_map = None
        for pat in ('gz sim', 'navigation.launch', 'parameter_bridge', 'rviz2'):
            subprocess.run(['pkill', '-9', '-f', pat],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)

    # ---------------- save map ----------------
    def save_map(self):
        if self.saving:
            return
        path = self.map_path.get().strip()
        if not path:
            self.save_msg, self.save_color = 'enter a save path first', '#f38ba8'
            return
        self.saving = True
        self.save_msg, self.save_color = 'saving map\u2026', '#f9e2af'
        threading.Thread(target=self._save_worker, args=(path,), daemon=True).start()

    def _save_worker(self, path):
        try:
            r = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path],
                capture_output=True, text=True, timeout=30)
            if r.returncode == 0:
                self.save_msg = f'saved \u2713  {os.path.basename(path)}.yaml  (press \u21bb to list it)'
                self.save_color = '#a6e3a1'
                self.root.after(0, self.refresh_maps)
            else:
                tail = (r.stderr or r.stdout or 'unknown').strip().splitlines()[-1:]
                self.save_msg = 'save failed: ' + (tail[0] if tail else 'unknown')
                self.save_color = '#f38ba8'
        except subprocess.TimeoutExpired:
            self.save_msg, self.save_color = 'save timed out (is /map publishing?)', '#f38ba8'
        except Exception as e:
            self.save_msg, self.save_color = f'save error: {e}', '#f38ba8'
        finally:
            self.saving = False

    # ---------------- driving ----------------
    def forward(self):      self.node.vx = self.lin.get()
    def backward(self):     self.node.vx = -self.lin.get()
    def turn_left(self):    self.node.wz = self.ang.get()
    def turn_right(self):   self.node.wz = -self.ang.get()
    def strafe_left(self):  self.node.vy = self.lin.get()
    def strafe_right(self): self.node.vy = -self.lin.get()
    def stop(self):         self.node.zero()

    def on_key_press(self, event):
        # don't drive while typing in the save-path box
        if isinstance(event.widget, tk.Entry):
            return
        k = event.keysym.lower()
        job = self._release_jobs.pop(k, None)
        if job is not None:
            self.root.after_cancel(job)
        if k == 'space':
            self.pressed.clear()
        else:
            self.pressed.add(k)
        self.recompute()

    def on_key_release(self, event):
        if isinstance(event.widget, tk.Entry):
            return
        k = event.keysym.lower()
        job = self._release_jobs.pop(k, None)
        if job is not None:
            self.root.after_cancel(job)
        self._release_jobs[k] = self.root.after(60, lambda: self._do_release(k))

    def _do_release(self, k):
        self._release_jobs.pop(k, None)
        self.pressed.discard(k)
        self.recompute()

    def recompute(self):
        lin, ang = self.lin.get(), self.ang.get()
        vx = vy = wz = 0.0
        if 'w' in self.pressed: vx += lin
        if 's' in self.pressed: vx -= lin
        if 'a' in self.pressed: wz += ang
        if 'd' in self.pressed: wz -= ang
        if 'q' in self.pressed: vy += lin
        if 'e' in self.pressed: vy -= lin
        self.node.vx, self.node.vy, self.node.wz = vx, vy, wz

    # ---------------- main loop ----------------
    def tick(self):
        if self.node.vx or self.node.vy or self.node.wz:
            self._zero_burst = 5
            self.node.publish()
        elif self._zero_burst > 0:
            self._zero_burst -= 1
            self.node.publish()

        running = self.proc is not None and self.proc.poll() is None
        if self.mode != 'stopped' and not running and not self.busy:
            self.status_msg = f'{self.mode.upper()} stack exited - check {LOGDIR}'
            self.status_color = '#f38ba8'
            self.mode = 'stopped'
        self.status.config(text=self.status_msg, fg=self.status_color)
        if self.save_msg:
            self.save_status.config(text=self.save_msg, fg=self.save_color)
        self.drive_status.config(
            text=f'vx={self.node.vx:+.2f}  vy={self.node.vy:+.2f}  wz={self.node.wz:+.2f}')

        norm = 'disabled' if self.busy else 'normal'
        self.btn_slam.config(state=norm)
        self.btn_map.config(state=norm)
        self.btn_stop.config(state=norm)
        self.btn_del.config(state=norm)
        self.btn_save.config(state='disabled' if self.saving else 'normal',
                             text='saving\u2026' if self.saving else 'Save Map')
        self.root.after(int(1000 / PUBLISH_HZ), self.tick)

    def on_close(self):
        self.node.zero()
        for _ in range(5):
            self.node.publish()
        self._stop_proc()
        self.root.destroy()


def main():
    rclpy.init()
    node = TeleopNode()
    root = tk.Tk()
    Panel(root, node)
    try:
        root.mainloop()
    finally:
        node.zero()
        node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
