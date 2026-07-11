/// Process using tiled rendering for high-resolution output (e.g., 4K→4K).
    /// Splits the input into tiles with overlap, processes each tile independently,
    /// and stitches the results together.
    pub fn process_tiled(
        &mut self,
        raw_data: &[u8],
        width: u32,
        height: u32,
        warp_params: &GpuWarpParams,
    ) -> crate::error::IspResult<IspFrame> {
        if !self.initialized {
            return Err(crate::error::IspError::Config("pipeline not initialized".into()));
        }

        // Check if tiled rendering is enabled in profile
        if !self.config.profile.use_tiled_rendering {
            return self.process_with_warp(raw_data, width, height, warp_params);
        }

        let tile_x = self.config.profile.tile_count_x.max(1) as u32;
        let tile_y = self.config.profile.tile_count_y.max(1) as u32;
        let overlap = self.config.profile.tile_overlap as u32;

        if tile_x == 1 && tile_y == 1 {
            return self.process_with_warp(raw_data, width, height, warp_params);
        }

        info!("Tiled rendering: {}×{} tiles with {}px overlap", tile_x, tile_y, overlap);

        // Calculate tile dimensions with overlap
        let tile_w = (width + tile_x - 1) / tile_x;
        let tile_h = (height + tile_y - 1) / tile_y;

        // Output dimensions per tile (without overlap)
        let out_tile_w = self.width / tile_x;
        let out_tile_h = self.height / tile_y;

        // Total output float data size: [1, 3, H, W]
        let total_float_size = (self.width * self.height * 3) as usize;
        let mut stitched_float = vec![0.0f32; total_float_size];

        // Process each tile
        for ty in 0..tile_y {
            for tx in 0..tile_x {
                // Input tile bounds with overlap
                let in_x_start = (tx * tile_w).saturating_sub(overlap);
                let in_y_start = (ty * tile_h).saturating_sub(overlap);
                let in_x_end = ((tx + 1) * tile_w).min(width) + overlap;
                let in_y_end = ((ty + 1) * tile_h).min(height) + overlap;

                let tile_in_w = in_x_end - in_x_start;
                let tile_in_h = in_y_end - in_y_start;

                // Output tile bounds (without overlap)
                let out_x_start = tx * out_tile_w;
                let out_y_start = ty * out_tile_h;
                let tile_out_w = out_tile_w;
                let tile_out_h = out_tile_h;

                info!("Processing tile ({},{}): in={}x{} out={}x{} at ({},{}) input offset=({},{})",
                    tx, ty, tile_in_w, tile_in_h, tile_out_w, tile_out_h, out_x_start, out_y_start, in_x_start, in_y_start);

                // Extract tile from raw data
                let tile_raw = Self::extract_tile(raw_data, width, in_x_start, in_y_start, tile_in_w, tile_in_h);

                // Process tile
                let mut params = ProcessParams::new(tile_in_w, tile_in_h, &tile_raw);
                params.isp_params = Some(self.controller.analyze_and_update(&crate::pipeline::types::IspFrame {
                    params: crate::isp_params::IspParams::default(),
                    data: tile_raw.clone(),
                    width: tile_in_w,
                    height: tile_in_h,
                    format: cam_types::FrameFormat::RawSensor,
                    float_data: None,
                    aux: None,
                    timestamp_ns: 0,
                    prep_duration_ns: 0,
                    inference_duration_ns: 0,
                    total_duration_ns: 0,
                }).clone());
                params.target_width = tile_out_w;
                params.target_height = tile_out_h;
                params.sensor_max = self.config.sensor_max;
                params.output_format = self.config.output_format;

                let mut tile_output = self.engine.process(&params)?;

                // Apply GPU warp per-tile if enabled
                #[cfg(feature = "mnn")]
                if self.config.gpu_warp_enabled && warp_params.needs_warp() {
                    if let Some(ref warp_engine) = self.gpu_warp_engine {
                        if let Some(ref mut float_data) = tile_output.float_data {
                            let n = (tile_out_w * tile_out_h * 3) as usize;
                            if float_data.len() >= n {
                                let mut input_copy = vec![0.0f32; n];
                                input_copy.copy_from_slice(&float_data[..n]);
                                let _ = warp_engine.run_into(
                                    &input_copy,
                                    warp_params.gdc_k1,
                                    warp_params.gdc_k2,
                                    warp_params.gdc_k3,
                                    warp_params.eis_dx,
                                    warp_params.eis_dy,
                                    &mut float_data[..n],
                                );
                            }
                        }
                    }
                }

                // Stitch tile float data into output
                if let Some(ref tile_float) = tile_output.float_data {
                    Self::stitch_tile_f32(
                        &mut stitched_float,
                        tile_float,
                        self.width,
                        self.height,
                        out_x_start,
                        out_y_start,
                        tile_out_w,
                        tile_out_h,
                    );
                }
            }
        }

        // Create stitched ISP output
        let mut isp_output = IspFrame {
            params: crate::isp_params::IspParams::default(),
            float_data: Some(stitched_float),
            width: self.width,
            height: self.height,
            format: cam_types::FrameFormat::NchwFloat,
            data: vec![],
            aux: None,
            timestamp_ns: 0,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        };

        // Apply GPU warp on stitched result if needed
        #[cfg(feature = "mnn")]
        if self.config.gpu_warp_enabled && warp_params.needs_warp() {
            if let Some(ref warp_engine) = self.gpu_warp_engine {
                if let Some(ref mut float_data) = isp_output.float_data {
                    let n = (self.width * self.height * 3) as usize;
                    if float_data.len() >= n {
                        let mut input_copy = vec![0.0f32; n];
                        input_copy.copy_from_slice(&float_data[..n]);
                        let _ = warp_engine.run_into(
                            &input_copy,
                            warp_params.gdc_k1,
                            warp_params.gdc_k2,
                            warp_params.gdc_k3,
                            warp_params.eis_dx,
                            warp_params.eis_dy,
                            &mut float_data[..n],
                        );
                    }
                }
            }
        }

        // Run post-processing on stitched result
        let post_output = if let Some(ref float_data) = isp_output.float_data {
            self.post_pipeline.process_float(
                float_data,
                isp_output.width,
                isp_output.height,
                isp_output.aux.clone(),
                isp_output.timestamp_ns,
                None,
            )?
        } else {
            self.post_pipeline.process(&isp_output)?
        };

        Ok(post_output)
    }
ENDOFFILE