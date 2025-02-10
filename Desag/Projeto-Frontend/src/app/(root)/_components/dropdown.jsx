import React from 'react';
import styles from './dropdown.module.css';

export default function Dropdown(props) {
  return (
    <div>
      {props.name && (
        <label className={styles.label}>{props.name}</label>
      )}
      <div className={styles.selectContainer}>
        <select
          value={props.value}
          onChange={(e) => props.onChange(e.target.value)}
          className={styles.input}
        >
          {props.list?.map((item, index) => (
            <option key={index} value={item.value}>
              {item.label}
            </option>
          ))}
        </select>
      </div>
    </div>
  );
}
