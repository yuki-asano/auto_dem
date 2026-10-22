#!/usr/bin/env roseus

;; Robot
(load "package://eus_assimp/euslisp/eus-assimp.l" )
(load "package://auto_dem/euslisp/fr3-dielec-interface.l")
(fr3-dielec-init)

;; Environment
(load "package://auto_dem/models/auto_dem.l")  ;; including dielectrometer and table
(auto_dem)
(setq *auto_dem_env* *auto_dem*)

;;;;
;; Parameters below are defined in urdf.

;; dielectrometer_table positioning
;;(send *dielectrometer_table* :move-to (send *auto_dem* :dielectrometer_table_body_lk :worldcoords))


;;;;;;;;;;;;;;;;;;
;; Coords for manipulation

;; generate dielectrometer manip coords
;; 10G apparatus
(setq *10G-insert-coords* (send *auto_dem* :10G_ip_lk :worldcoords))
(setq *10G-insert-coords-orig* (send *10G-insert-coords* :copy-worldcoords))
(setq *10G-manip-coords*
      (make-coords :pos (send *10G-insert-coords* :worldpos)
                   :rot (rotate-matrix (send *10G-insert-coords* :worldrot) 3.14 :z nil)
                   ))
;; 28G apparatus
(setq *28G-insert-coords* (send *auto_dem* :28G_ip_lk :worldcoords))
(setq *28G-insert-coords-orig* (send *28G-insert-coords* :copy-worldcoords))
(setq *28G-manip-coords*
      (make-coords :pos (send *28G-insert-coords* :worldpos)
                   :rot (rotate-matrix (send *28G-insert-coords* :worldrot) 3.14 :z nil)
                   ))
;; 40G apparatus
(setq *40G-insert-coords* (send *auto_dem* :40G_ip_lk :worldcoords))
(setq *40G-insert-coords-orig* (send *40G-insert-coords* :copy-worldcoords))
(setq *40G-manip-coords*
      (make-coords :pos (send *40G-insert-coords* :worldpos)
                   :rot (rotate-matrix (send *40G-insert-coords* :worldrot) 3.14 :z nil)
                   ))
;; polymer grasp point
(setq *polymer-gp-coords* (send *auto_dem* :polymer_grasp_point_lk :worldcoords))
(setq *polymer-gp-coords-orig* (send *polymer-gp-coords* :copy-worldcoords))
(setq *polymer-gp-manip-coords*
      (make-coords :pos (send *polymer-gp-coords* :worldpos)
                   :rot (rotate-matrix (send *polymer-gp-coords* :worldrot) 3.14 :z nil)
                   ))


(setq *display-objects* (list *robot*
                              (send *robot* :rarm :end-coords)
                              ;; env
                              *auto_dem*
                              *10G-manip-coords*
                              *28G-manip-coords*
                              *40G-manip-coords*
                              *polymer-gp-manip-coords*
                              ))

(objects *display-objects*)
